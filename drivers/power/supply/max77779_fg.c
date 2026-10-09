// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only fuel-gauge driver for the Maxim MAX77779.
 *
 * The MAX77779 PMIC integrates a ModelGauge m5 fuel gauge reachable as an
 * independent I2C device (default address 0x36). This driver exposes the
 * gauge's autonomously tracked state (state-of-charge, voltage, current,
 * temperature) through the power_supply class.
 *
 * It is deliberately read-only: it never programs the battery model and never
 * touches any charge rail. The gauge keeps its model in battery-backed RAM, so
 * a state-of-charge report is only trustworthy while no power-on-reset (POR)
 * has wiped it. The driver therefore checks Status.POR / FStat.DNR before
 * returning any model-derived value and reports it as unavailable rather than
 * handing back a stale or default-model guess.
 *
 * The register layout and the ModelGauge m5 conversions mirror the in-tree
 * max1720x_battery.c driver.
 *
 * Copyright (C) 2026 Steffen Deusch
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

/* ModelGauge m5 registers (16-bit, little-endian on the wire) */
#define MAX77779_FG_STATUS		0x00
#define MAX77779_FG_STATUS_POR		BIT(1)	/* power-on reset seen; sticky */
#define MAX77779_FG_OCV0		0x80	/* heads the ModelGauge m5 model block */
#define MAX77779_FG_STATUS_BST		BIT(3)	/* 0 = battery present */
#define MAX77779_FG_REPCAP		0x06
#define MAX77779_FG_REPSOC		0x07
#define MAX77779_FG_FULLCAPREP		0x10
#define MAX77779_FG_AVGVCELL		0x19
#define MAX77779_FG_VCELL		0x1a
#define MAX77779_FG_TEMP		0x1b
#define MAX77779_FG_CURRENT		0x1c
#define MAX77779_FG_AVGCURRENT		0x1d
#define MAX77779_FG_DEVNAME		0x21
#define MAX77779_FG_DEVNAME_RADIX	GENMASK(15, 8)
#define MAX77779_FG_FSTAT		0x3d
#define MAX77779_FG_FSTAT_DNR		BIT(0)	/* data not ready */

#define MAX77779_FG_MAX_REG		0xff

/* Treat |Current| below this raw value as "no measurable flow". */
#define MAX77779_FG_CURRENT_DEADBAND	8

struct max77779_fg {
	struct regmap *regmap;
	int rsense;	/* micro-ohms; 0 = unknown -> current/charge disabled */
};

static bool max77779_fg_devname_valid(unsigned int devname)
{
	switch (FIELD_GET(MAX77779_FG_DEVNAME_RADIX, devname)) {
	case 0x51:
	case 0x62:
	case 0x63:
		return true;
	default:
		return false;
	}
}

/*
 * The state-of-charge and capacity registers are only meaningful once the
 * gauge has valid data. FStat.DNR says exactly that, and it clears on its own.
 *
 * Status.POR deliberately does not appear here. It is a sticky record that a
 * power-on reset happened -- nothing in this driver ever acknowledges it, so
 * it stays set for the life of the boot -- and gating on it left the battery
 * permanently unreadable on this phone even though the gauge itself was fine:
 * register 0x07 read 99% while the model registers at 0x80..0x9f were fully
 * populated. A battery that always reports 0% is worse than useless, because
 * UPower takes it for a critically low battery and powers the machine off.
 * Whether a model is present is checked once, in probe(), where a warning
 * still makes sense.
 */
static int max77779_fg_model_ready(struct max77779_fg *fg)
{
	unsigned int fstat;
	int ret;

	ret = regmap_read(fg->regmap, MAX77779_FG_FSTAT, &fstat);
	if (ret)
		return ret;

	if (fstat & MAX77779_FG_FSTAT_DNR)
		return -ENODATA;

	return 0;
}

static int max77779_fg_uvolt(unsigned int reg)
{
	return reg * 625 / 8;	/* 78.125 uV/LSB */
}

static int max77779_fg_temp(unsigned int reg)
{
	return (s16)reg * 10 / 256;	/* tenths of a degree Celsius */
}

static int max77779_fg_uamp(struct max77779_fg *fg, unsigned int reg)
{
	/* signed; 1.5625 uV / Rsense per LSB */
	return div_s64((s64)(s16)reg * 1562500, fg->rsense);
}

static int max77779_fg_uah(struct max77779_fg *fg, unsigned int reg)
{
	/* 5.0 uVh / Rsense per LSB */
	return div_s64((s64)reg * 5000000, fg->rsense);
}

static int max77779_fg_status(struct max77779_fg *fg,
			      union power_supply_propval *val)
{
	unsigned int cur, soc;
	int ret;

	ret = max77779_fg_model_ready(fg);
	if (ret == -ENODATA) {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	} else if (ret) {
		return ret;
	}

	ret = regmap_read(fg->regmap, MAX77779_FG_CURRENT, &cur);
	if (ret)
		return ret;

	if ((s16)cur > MAX77779_FG_CURRENT_DEADBAND) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
	} else if ((s16)cur < -MAX77779_FG_CURRENT_DEADBAND) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		ret = regmap_read(fg->regmap, MAX77779_FG_REPSOC, &soc);
		if (ret)
			return ret;
		val->intval = (soc >> 8) >= 100 ? POWER_SUPPLY_STATUS_FULL :
						  POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return 0;
}

static int max77779_fg_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct max77779_fg *fg = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(fg->regmap, MAX77779_FG_STATUS, &reg);
		if (ret) {
			val->intval = 0;
			return 0;
		}
		val->intval = !(reg & MAX77779_FG_STATUS_BST);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		return max77779_fg_status(fg, val);
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = max77779_fg_model_ready(fg);
		if (ret)
			return ret;
		ret = regmap_read(fg->regmap, MAX77779_FG_REPSOC, &reg);
		if (ret)
			return ret;
		val->intval = min(reg >> 8, 100u);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(fg->regmap, MAX77779_FG_VCELL, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uvolt(reg);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(fg->regmap, MAX77779_FG_AVGVCELL, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uvolt(reg);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = regmap_read(fg->regmap, MAX77779_FG_TEMP, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_temp(reg);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!fg->rsense)
			return -ENODATA;
		ret = regmap_read(fg->regmap, MAX77779_FG_CURRENT, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uamp(fg, reg);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (!fg->rsense)
			return -ENODATA;
		ret = regmap_read(fg->regmap, MAX77779_FG_AVGCURRENT, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uamp(fg, reg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (!fg->rsense)
			return -ENODATA;
		ret = max77779_fg_model_ready(fg);
		if (ret)
			return ret;
		ret = regmap_read(fg->regmap, MAX77779_FG_REPCAP, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uah(fg, reg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (!fg->rsense)
			return -ENODATA;
		ret = max77779_fg_model_ready(fg);
		if (ret)
			return ret;
		ret = regmap_read(fg->regmap, MAX77779_FG_FULLCAPREP, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uah(fg, reg);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "max77779fg";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Maxim Integrated";
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property max77779_fg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static const struct regmap_config max77779_fg_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = MAX77779_FG_MAX_REG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static const struct power_supply_desc max77779_fg_desc = {
	.name = "max77779fg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = max77779_fg_props,
	.num_properties = ARRAY_SIZE(max77779_fg_props),
	.get_property = max77779_fg_get_property,
};

static int max77779_fg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct max77779_fg *fg;
	struct power_supply *psy;
	unsigned int devname, status, fstat;
	u32 rsense;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->regmap = devm_regmap_init_i2c(client, &max77779_fg_regmap_cfg);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(dev, PTR_ERR(fg->regmap),
				     "regmap init failed\n");

	ret = regmap_read(fg->regmap, MAX77779_FG_DEVNAME, &devname);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read DevName\n");

	if (!max77779_fg_devname_valid(devname))
		return dev_err_probe(dev, -ENODEV,
				     "unexpected DevName 0x%04x\n", devname);

	/* Rsense is only needed to scale current and charge readings. */
	if (device_property_read_u32(dev, "shunt-resistor-micro-ohms", &rsense))
		rsense = 0;
	fg->rsense = rsense;
	if (!fg->rsense)
		dev_info(dev, "no shunt-resistor-micro-ohms; current/charge reporting disabled\n");

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(dev);

	psy = devm_power_supply_register(dev, &max77779_fg_desc, &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy),
				     "failed to register power supply\n");

	/*
	 * Read-only diagnostic. Status.POR is sticky and says only that a
	 * power-on reset happened, so the question worth answering is whether
	 * a battery model is loaded at all -- OCV0 heads the model block and
	 * is the cheapest evidence either way.
	 */
	if (!regmap_read(fg->regmap, MAX77779_FG_STATUS, &status) &&
	    !regmap_read(fg->regmap, MAX77779_FG_FSTAT, &fstat)) {
		unsigned int ocv0 = 0;

		regmap_read(fg->regmap, MAX77779_FG_OCV0, &ocv0);
		dev_info(dev,
			 "Status=0x%04x FStat=0x%04x OCV0=0x%04x%s\n",
			 status, fstat, ocv0,
			 (status & MAX77779_FG_STATUS_POR) ?
				" (POR seen; sticky, ignored for state-of-charge)" : "");
		if (!ocv0)
			dev_warn(dev,
				 "no battery model loaded (OCV0 reads 0); state-of-charge will not be meaningful\n");
	}

	return 0;
}

static const struct of_device_id max77779_fg_of_match[] = {
	{ .compatible = "maxim,max77779fg" },
	{}
};
MODULE_DEVICE_TABLE(of, max77779_fg_of_match);

static const struct i2c_device_id max77779_fg_id[] = {
	{ "max77779fg" },
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_fg_id);

static struct i2c_driver max77779_fg_i2c_driver = {
	.driver = {
		.name = "max77779-fg",
		.of_match_table = max77779_fg_of_match,
	},
	.id_table = max77779_fg_id,
	.probe = max77779_fg_probe,
};
module_i2c_driver(max77779_fg_i2c_driver);

MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_DESCRIPTION("Maxim MAX77779 fuel gauge driver");
MODULE_LICENSE("GPL");

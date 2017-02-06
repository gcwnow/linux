/*
 * Configuration and power control of LCD panel controlled by Novatek NT39016.
 *
 * Copyright (C) 2017, Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>

#include <asm/mach-jz4740/gpio.h>


struct nt39016_platform_data {
        int gpio_reset;
        int gpio_clock;
        int gpio_enable;
        int gpio_data;
};

static struct nt39016_platform_data gcw0_panel_pdata = {
	.gpio_reset		= JZ_GPIO_PORTE(2),
	.gpio_clock		= JZ_GPIO_PORTE(15),
	.gpio_enable		= JZ_GPIO_PORTE(16),
	.gpio_data		= JZ_GPIO_PORTE(17),
};

#define GPIO_PANEL_SOMETHING	JZ_GPIO_PORTF(0)

struct nt39016 {
	struct device *dev;
	struct regmap *regmap;
	struct lcd_device *lcd;

	struct regulator *gcw0_lcd_regulator_33v;
	struct regulator *gcw0_lcd_regulator_18v;

	unsigned int powerdown:1;
	unsigned int suspended:1;
};

#define RV(REG, VAL) { .reg = (REG), .def = (VAL), .delay_us = 2 }
static const struct reg_sequence nt39016_panel_regs[] = {
	RV(0x00, 0x07), RV(0x01, 0x00), RV(0x02, 0x03), RV(0x03, 0xCC),
	RV(0x04, 0x46), RV(0x05, 0x05), RV(0x06, 0x00), RV(0x07, 0x00),
	RV(0x08, 0x08), RV(0x09, 0x40), RV(0x0A, 0x88), RV(0x0B, 0x88),
	RV(0x0C, 0x20), RV(0x0D, 0x20), RV(0x0E, 0x67), RV(0x0F, 0xA4),
	RV(0x10, 0x04), RV(0x11, 0x24), RV(0x12, 0x24), RV(0x20, 0x00),
};
#undef RV

static int nt39016_reg_write(void *context, unsigned int reg, unsigned int val)
{
	//struct nt39016 *nt39016 = context;
	struct nt39016_platform_data *pdata = &gcw0_panel_pdata;
	unsigned int data = (reg << 10) | (1 << 9) | val;
	int bit;

	gpio_direction_output(pdata->gpio_enable, 0);

	for (bit = 15; bit >= 0; bit--) {
		gpio_direction_output(pdata->gpio_clock, 0);
		gpio_direction_output(pdata->gpio_data, (data >> bit) & 1);
		udelay(1);
		gpio_direction_output(pdata->gpio_clock, 1);
		udelay(1);
	}

	gpio_direction_output(pdata->gpio_enable, 1);

	/* Note: Both clock and enable pin are left in inactive state (1). */

	return 0;
}

static const struct regmap_range nt39016_regmap_no_ranges[] = {
	regmap_reg_range(0x13, 0x1D),
	regmap_reg_range(0x1F, 0x1F),
};

static const struct regmap_access_table nt39016_regmap_access_table = {
	.no_ranges = nt39016_regmap_no_ranges,
	.n_no_ranges = ARRAY_SIZE(nt39016_regmap_no_ranges),
};

static const struct regmap_config nt39016_regmap_config = {
	.reg_bits = 6,
	.val_bits = 8,

	// TODO: All registers are readable, but we haven't implemented reading yet.
	.reg_write = nt39016_reg_write,
	.max_register = 0x20,
	.wr_table = &nt39016_regmap_access_table,
	//.rd_table = &nt39016_regmap_access_table,

	.cache_type = REGCACHE_FLAT,
};

static int nt39016_power_up(struct nt39016 *nt39016)
{
	struct device *dev = nt39016->dev;
	struct nt39016_platform_data *pdata = &gcw0_panel_pdata;
	int err;

	err = regulator_enable(nt39016->gcw0_lcd_regulator_33v);
	if (err) {
		dev_err(dev, "Failed to enable 3.3V regulator: %d\n", err);
		return err;
	}

	err = regulator_enable(nt39016->gcw0_lcd_regulator_18v);
	if (err) {
		dev_err(dev, "Failed to enable 1.8V regulator: %d\n", err);
		return err;
	}

	/* Reset LCD panel. */
	gpio_direction_output(pdata->gpio_reset, 0);
	udelay(50);
	gpio_direction_output(pdata->gpio_reset, 1);
	udelay(2);

	/* Init panel registers. */
	err = regmap_multi_reg_write(nt39016->regmap,
					nt39016_panel_regs,
					ARRAY_SIZE(nt39016_panel_regs));
	if (err) {
		dev_err(dev, "Failed to write registers: %d\n", err);
		return err;
	}

	return 0;
}

static int nt39016_power_down(struct nt39016 *nt39016)
{
	struct device *dev = nt39016->dev;
	int err;

	err = regmap_write(nt39016->regmap, 0x00, 0x05);
	if (err) {
		dev_err(dev, "Failed to write registers: %d\n", err);
		return err;
	}

	err = regulator_disable(nt39016->gcw0_lcd_regulator_18v);
	if (err) {
		dev_err(dev, "Failed to disable 1.8V regulator: %d\n", err);
		return err;
	}

	err = regulator_disable(nt39016->gcw0_lcd_regulator_33v);
	if (err) {
		dev_err(dev, "Failed to disable 3.3V regulator: %d\n", err);
		return err;
	}

	return 0;
}

static int nt39016_get_power(struct lcd_device *lcd)
{
	struct nt39016 *nt39016 = lcd_get_data(lcd);

	return nt39016->powerdown ? FB_BLANK_POWERDOWN : FB_BLANK_UNBLANK;
}

static int nt39016_set_power(struct lcd_device *lcd, int power)
{
	struct nt39016 *nt39016 = lcd_get_data(lcd);
	unsigned int powerdown = (power == FB_BLANK_POWERDOWN);

	if (powerdown == nt39016->powerdown)
		return 0;

	if (!nt39016->suspended) {
		int err;
		if (powerdown)
			err = nt39016_power_down(nt39016);
		else
			err = nt39016_power_up(nt39016);
		if (err)
			return err;
	}

	nt39016->powerdown = powerdown;

	return 0;
}

static int nt39016_match(struct lcd_device *lcd, struct fb_info *info)
{
	struct nt39016 *nt39016 = lcd_get_data(lcd);

	return nt39016->dev->parent == info->device;
}

static struct lcd_ops nt39016_ops = {
	.get_power	= nt39016_get_power,
	.set_power	= nt39016_set_power,
	.check_fb	= nt39016_match,
};

static int nt39016_probe(struct platform_device *pdev)
{
	struct nt39016 *nt39016;
	struct device *dev = &pdev->dev;
	struct nt39016_platform_data *pdata = &gcw0_panel_pdata;
	struct regulator *gcw0_lcd_regulator_33v;
	struct regulator *gcw0_lcd_regulator_18v;
	int err;

	gcw0_lcd_regulator_33v = devm_regulator_get_optional(dev, "LDO6");
	if (IS_ERR(gcw0_lcd_regulator_33v)) {
		err = PTR_ERR(gcw0_lcd_regulator_33v);
		if (err == -ENODEV) {
			return -EPROBE_DEFER;
		} else {
			dev_err(dev, "Regulator LD06 missing: %d\n", err);
			return err;
		}
	}

	gcw0_lcd_regulator_18v = devm_regulator_get_optional(dev, "LDO8");
	if (IS_ERR(gcw0_lcd_regulator_18v)) {
		err = PTR_ERR(gcw0_lcd_regulator_18v);
		if (err == -ENODEV) {
			return -EPROBE_DEFER;
		} else {
			dev_err(dev, "Regulator LD08 missing: %d\n", err);
			return err;
		}
	}

	/* Reserve GPIO pins. */

	err = devm_gpio_request(dev, pdata->gpio_reset, "LCD panel reset");
	if (err) {
		dev_err(dev,
			"Failed to request LCD panel reset pin: %d\n", err);
		return err;
	}

	err = devm_gpio_request(dev, pdata->gpio_clock, "LCD 3-wire clock");
	if (err) {
		dev_err(dev,
			"Failed to request LCD panel 3-wire clock pin: %d\n",
			err);
		return err;
	}

	err = devm_gpio_request(dev, pdata->gpio_enable, "LCD 3-wire enable");
	if (err) {
		dev_err(dev,
			"Failed to request LCD panel 3-wire enable pin: %d\n",
			err);
		return err;
	}

	err = devm_gpio_request(dev, pdata->gpio_data, "LCD 3-wire data");
	if (err) {
		dev_err(dev,
			"Failed to request LCD panel 3-wire data pin: %d\n",
			err);
		return err;
	}

	/* Set initial GPIO pin directions and value. */

	gpio_direction_output(pdata->gpio_clock,  1);
	gpio_direction_output(pdata->gpio_enable, 1);
	gpio_direction_output(pdata->gpio_data,   0);

	err = devm_gpio_request(dev, GPIO_PANEL_SOMETHING, "LCD panel unknown");
	if (err) {
		dev_warn(dev,
			"Failed to request LCD panel unknown pin: %d\n", err);
	} else {
		gpio_direction_output(GPIO_PANEL_SOMETHING, 1);
	}

	nt39016 = devm_kzalloc(dev, sizeof(*nt39016), GFP_KERNEL);
	if (!nt39016)
		return -ENOMEM;

	nt39016->dev = dev;
	nt39016->gcw0_lcd_regulator_33v = gcw0_lcd_regulator_33v;
	nt39016->gcw0_lcd_regulator_18v = gcw0_lcd_regulator_18v;
	platform_set_drvdata(pdev, nt39016);

	nt39016->regmap = devm_regmap_init(dev, NULL, nt39016,
					   &nt39016_regmap_config);
	if (IS_ERR(nt39016->regmap)) {
		err = PTR_ERR(nt39016->regmap);
		dev_err(dev, "Failed to init regmap: %d\n", err);
		return err;
	}

	nt39016->lcd = devm_lcd_device_register(dev, dev_name(dev), dev,
						nt39016, &nt39016_ops);
	if (IS_ERR(nt39016->lcd)) {
		err = PTR_ERR(nt39016->lcd);
		dev_err(dev, "Failed to register LCD device: %d\n", err);
		return err;
	}

	nt39016_power_up(nt39016);

	return 0;
}

static int nt39016_remove(struct platform_device *pdev)
{
	struct nt39016 *nt39016 = platform_get_drvdata(pdev);

	if (!nt39016->powerdown)
		nt39016_power_down(nt39016);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int nt39016_suspend(struct device *dev)
{
	struct nt39016 *nt39016 = dev_get_drvdata(dev);

	nt39016->suspended = 1;
	if (!nt39016->powerdown)
		nt39016_power_down(nt39016);

	return 0;
}

static int nt39016_resume(struct device *dev)
{
	struct nt39016 *nt39016 = dev_get_drvdata(dev);

	nt39016->suspended = 0;
	if (!nt39016->powerdown)
		nt39016_power_up(nt39016);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(nt39016_pm_ops, nt39016_suspend, nt39016_resume);

#ifdef CONFIG_OF
static const struct of_device_id nt39016_of_match[] = {
	{ .compatible = "novatek,nt39016" },
	{},
};
MODULE_DEVICE_TABLE(of, nt39016_of_match);
#endif

static struct platform_driver nt39016_driver = {
	.driver		= {
		.name		= "nt39016",
		.pm		= &nt39016_pm_ops,
		.of_match_table	= of_match_ptr(nt39016_of_match),
	},
	.probe		= nt39016_probe,
	.remove		= nt39016_remove,
};

module_platform_driver(nt39016_driver);

MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:nt39016");

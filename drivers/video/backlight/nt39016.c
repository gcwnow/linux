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
#include <linux/gpio/consumer.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>

#include <asm/mach-jz4740/gpio.h>


struct nt39016 {
	struct device *dev;
	struct device_node *fb_node;
	struct regmap *regmap;
	struct lcd_device *lcd;

	struct regulator *panel_regulator;

	unsigned int powerdown:1;
	unsigned int suspended:1;
};

#define RV(REG, VAL) { .reg = (REG), .def = (VAL), .delay_us = 2 }
static const struct reg_sequence nt39016_panel_regs[] = {
	RV(0x00, 0x04), RV(0x01, 0x00), RV(0x02, 0x03), RV(0x03, 0xCC),
	RV(0x04, 0x46), RV(0x05, 0x05), RV(0x06, 0x00), RV(0x07, 0x00),
	RV(0x08, 0x08), RV(0x09, 0x40), RV(0x0A, 0x88), RV(0x0B, 0x88),
	RV(0x0C, 0x20), RV(0x0D, 0x20), RV(0x0E, 0x67), RV(0x0F, 0xA4),
	RV(0x10, 0x04), RV(0x11, 0x24), RV(0x12, 0x24), RV(0x20, 0x00),
};
#undef RV

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
	.pad_bits = 2,
	.val_bits = 8,

	// TODO: All registers are readable, but we haven't implemented reading yet.
	.max_register = 0x20,
	.wr_table = &nt39016_regmap_access_table,
	//.rd_table = &nt39016_regmap_access_table,
	.write_flag_mask = 0x02,

	.cache_type = REGCACHE_FLAT,
};

static int nt39016_write_reg(struct nt39016 *nt39016,
			     unsigned int reg, unsigned int val)
{
	int err = regmap_write(nt39016->regmap, reg, val);
	if (err)
		dev_err(nt39016->dev, "Failed to write register: %d\n", err);
	return err;
}

static int nt39016_power_up(struct nt39016 *nt39016)
{
	struct device *dev = nt39016->dev;
	int err;

	err = regulator_enable(nt39016->panel_regulator);
	if (err) {
		dev_err(dev, "Failed to enable LCD panel regulator: %d\n", err);
		return err;
	}

	err = nt39016_write_reg(nt39016, 0x00, 0x07);
	if (err)
		return err;

	return 0;
}

static int nt39016_power_down(struct nt39016 *nt39016)
{
	struct device *dev = nt39016->dev;
	int err;

	err = nt39016_write_reg(nt39016, 0x00, 0x05);
	if (err)
		return err;

	err = regulator_disable(nt39016->panel_regulator);
	if (err) {
		dev_err(dev, "Failed to disable LCD panel regulator: %d\n", err);
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

static int nt39016_get_contrast(struct lcd_device *lcd)
{
	struct nt39016 *nt39016 = lcd_get_data(lcd);
	unsigned int val;
	int err;

	err = regmap_read(nt39016->regmap, 0x08, &val);
	if (err)
		return err;

	return (int)(val & 0x1F);
}

static int nt39016_set_contrast(struct lcd_device *lcd, int contrast)
{
	struct nt39016 *nt39016 = lcd_get_data(lcd);
	int err;

	if (contrast > 0x1F)
		contrast = 0x1F;

	err = nt39016_write_reg(nt39016, 0x08, (unsigned int)contrast);
	if (err)
		return err;

	return 0;
}

static int nt39016_match(struct lcd_device *lcd, struct fb_info *info)
{
	struct nt39016 *nt39016 = lcd_get_data(lcd);

	return !nt39016->fb_node || info->device->of_node == nt39016->fb_node;
}

static struct lcd_ops nt39016_ops = {
	.get_power	= nt39016_get_power,
	.set_power	= nt39016_set_power,
	.get_contrast	= nt39016_get_contrast,
	.set_contrast	= nt39016_set_contrast,
	.check_fb	= nt39016_match,
};

static int nt39016_probe(struct spi_device *spi)
{
	struct nt39016 *nt39016;
	struct device *dev = &spi->dev;
	struct gpio_desc *reset;
	struct regulator *panel_regulator;
	int err;

	panel_regulator = devm_regulator_get_optional(dev, "panel");
	if (IS_ERR(panel_regulator)) {
		err = PTR_ERR(panel_regulator);
		if (err == -ENODEV) {
			return -EPROBE_DEFER;
		} else {
			dev_err(dev, "LCD panel regulator missing: %d\n", err);
			return err;
		}
	}

	/*
	 * Acquire reset GPIO pin and reset the NT39016.
	 * The documentation says the reset pulse should be at least 40 us to
	 * pass the glitch filter, but when testing I see some resets fail and
	 * some succeed when using a 70 us delay, so we use 100 us instead.
	 */
	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset)) {
		err = PTR_ERR(reset);
		if (err == -EPROBE_DEFER)
			return err;
		dev_warn(dev, "Failed to get reset pin: %d\n", err);
		reset = NULL;
	}
	if (reset) {
		usleep_range(100, 1000);
		gpiod_set_value_cansleep(reset, 0);
		udelay(2);
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	err = spi_setup(spi);
	if (err)
		return err;

	nt39016 = devm_kzalloc(dev, sizeof(*nt39016), GFP_KERNEL);
	if (!nt39016)
		return -ENOMEM;

	nt39016->dev = dev;
	nt39016->panel_regulator = panel_regulator;
	spi_set_drvdata(spi, nt39016);

	nt39016->fb_node = of_parse_phandle(dev->of_node, "fb-dev", 0);
	if (nt39016->fb_node)
		dev_info(dev, "Listening to framebuffer device %s\n",
			 nt39016->fb_node->name);
	else
		dev_info(dev, "Listening to any framebuffer device\n");

	nt39016->regmap = devm_regmap_init_spi(spi, &nt39016_regmap_config);
	if (IS_ERR(nt39016->regmap)) {
		err = PTR_ERR(nt39016->regmap);
		dev_err(dev, "Failed to init regmap: %d\n", err);
		return err;
	}

	/* Init all registers. */
	err = regmap_multi_reg_write(nt39016->regmap, nt39016_panel_regs,
				     ARRAY_SIZE(nt39016_panel_regs));
	if (err) {
		dev_err(dev, "Failed to init registers: %d\n", err);
		return err;
	}

	nt39016_power_up(nt39016);

	nt39016->lcd = devm_lcd_device_register(dev, dev_name(dev), dev,
						nt39016, &nt39016_ops);
	if (IS_ERR(nt39016->lcd)) {
		err = PTR_ERR(nt39016->lcd);
		dev_err(dev, "Failed to register LCD device: %d\n", err);
		return err;
	}

	nt39016->lcd->props.max_contrast = 0x1F;

	return 0;
}

static int nt39016_remove(struct spi_device *spi)
{
	struct nt39016 *nt39016 = spi_get_drvdata(spi);

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

static struct spi_driver nt39016_driver = {
	.driver		= {
		.name		= "nt39016",
		.pm		= &nt39016_pm_ops,
		.of_match_table	= of_match_ptr(nt39016_of_match),
	},
	.probe		= nt39016_probe,
	.remove		= nt39016_remove,
};

module_spi_driver(nt39016_driver);

MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:nt39016");

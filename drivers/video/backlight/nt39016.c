
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/regmap.h>

#include <video/jzpanel.h>
#include <video/panel-nt39016.h>


struct nt39016 {
	struct nt39016_platform_data *pdata;
	struct regmap *regmap;
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
	struct nt39016 *panel = context;
	struct nt39016_platform_data *pdata = panel->pdata;
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

static int nt39016_panel_init(void **out_panel, struct device *dev,
			      void *panel_pdata)
{
	struct nt39016_platform_data *pdata = panel_pdata;
	struct nt39016 *panel;
	int ret;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel) {
		dev_err(dev, "Failed to alloc panel data\n");
		return -ENOMEM;
	}

	panel->pdata = pdata;

	/* Reserve GPIO pins. */

	ret = devm_gpio_request(dev, pdata->gpio_reset, "LCD panel reset");
	if (ret) {
		dev_err(dev,
			"Failed to request LCD panel reset pin: %d\n", ret);
		return ret;
	}

	ret = devm_gpio_request(dev, pdata->gpio_clock, "LCD 3-wire clock");
	if (ret) {
		dev_err(dev,
			"Failed to request LCD panel 3-wire clock pin: %d\n",
			ret);
		return ret;
	}

	ret = devm_gpio_request(dev, pdata->gpio_enable, "LCD 3-wire enable");
	if (ret) {
		dev_err(dev,
			"Failed to request LCD panel 3-wire enable pin: %d\n",
			ret);
		return ret;
	}

	ret = devm_gpio_request(dev, pdata->gpio_data, "LCD 3-wire data");
	if (ret) {
		dev_err(dev,
			"Failed to request LCD panel 3-wire data pin: %d\n",
			ret);
		return ret;
	}

	/* Set initial GPIO pin directions and value. */

	gpio_direction_output(pdata->gpio_clock,  1);
	gpio_direction_output(pdata->gpio_enable, 1);
	gpio_direction_output(pdata->gpio_data,   0);

	panel->regmap = devm_regmap_init(dev, NULL, panel,
					 &nt39016_regmap_config);
	if (IS_ERR(panel->regmap))
		return PTR_ERR(panel->regmap);

	*out_panel = panel;

	return 0;
}

static void nt39016_panel_exit(void *panel)
{
}

static void nt39016_panel_enable(void *panel)
{
	struct nt39016_platform_data *pdata = ((struct nt39016 *)panel)->pdata;
	struct regmap *regmap = ((struct nt39016 *)panel)->regmap;
	int err;

	/* Reset LCD panel. */
	gpio_direction_output(pdata->gpio_reset, 0);
	udelay(50);
	gpio_direction_output(pdata->gpio_reset, 1);
	udelay(2);

	/* Init panel registers. */
	err = regmap_multi_reg_write(regmap, nt39016_panel_regs,
				     ARRAY_SIZE(nt39016_panel_regs));
	if (err)
		printk("nt39016_panel_enable failed: %d\n", err);
}

static void nt39016_panel_disable(void *panel)
{
	struct regmap *regmap = ((struct nt39016 *)panel)->regmap;

	regmap_write(regmap, 0x00, 0x05);
}

struct panel_ops nt39016_panel_ops = {
	.init		= nt39016_panel_init,
	.exit		= nt39016_panel_exit,
	.enable		= nt39016_panel_enable,
	.disable	= nt39016_panel_disable,
};

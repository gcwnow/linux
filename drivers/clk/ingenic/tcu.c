// SPDX-License-Identifier: GPL-2.0
/*
 * Ingenic JZ47xx SoC TCU clocks driver
 * Copyright (C) 2018 Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/ingenic-tcu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/ingenic,tcu.h>

enum ingenic_version {
	ID_JZ4740,
	ID_JZ4770,
	ID_JZ4780,
};

struct ingenic_tcu_clk_info {
	struct clk_init_data init_data;
	u8 gate_bit;
	u8 tcsr_reg;
};

struct ingenic_tcu_clk {
	struct clk_hw hw;

	struct regmap *map;
	const struct ingenic_tcu_clk_info *info;

	unsigned int idx;
};

#define to_tcu_clk(_hw) container_of(_hw, struct ingenic_tcu_clk, hw)

static int ingenic_tcu_enable(struct clk_hw *hw)
{
	struct ingenic_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct ingenic_tcu_clk_info *info = tcu_clk->info;

	regmap_write(tcu_clk->map, TCU_REG_TSCR, BIT(info->gate_bit));
	return 0;
}

static void ingenic_tcu_disable(struct clk_hw *hw)
{
	struct ingenic_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct ingenic_tcu_clk_info *info = tcu_clk->info;

	regmap_write(tcu_clk->map, TCU_REG_TSSR, BIT(info->gate_bit));
}

static int ingenic_tcu_is_enabled(struct clk_hw *hw)
{
	struct ingenic_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct ingenic_tcu_clk_info *info = tcu_clk->info;
	unsigned int value;

	regmap_read(tcu_clk->map, TCU_REG_TSR, &value);

	return !(value & BIT(info->gate_bit));
}

static u8 ingenic_tcu_get_parent(struct clk_hw *hw)
{
	struct ingenic_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct ingenic_tcu_clk_info *info = tcu_clk->info;
	unsigned int val = 0;
	int ret;

	ret = regmap_read(tcu_clk->map, info->tcsr_reg, &val);
	WARN_ONCE(ret < 0, "Unable to read TCSR %i", tcu_clk->idx);

	return ffs(val & TCU_TCSR_PARENT_CLOCK_MASK) - 1;
}

static int ingenic_tcu_set_parent(struct clk_hw *hw, u8 idx)
{
	struct ingenic_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct ingenic_tcu_clk_info *info = tcu_clk->info;
	struct regmap *map = tcu_clk->map;
	int ret;

	/*
	 * Our clock provider has the CLK_SET_PARENT_GATE flag set, so we know
	 * that the clk is in unprepared state. To be able to access TCSR
	 * we must ungate the clock supply and we gate it again when done.
	 */

	regmap_write(map, TCU_REG_TSCR, BIT(info->gate_bit));

	ret = regmap_update_bits(map, info->tcsr_reg,
			TCU_TCSR_PARENT_CLOCK_MASK, BIT(idx));
	WARN_ONCE(ret < 0, "Unable to update TCSR %i", tcu_clk->idx);

	regmap_write(map, TCU_REG_TSSR, BIT(info->gate_bit));

	return 0;
}

static unsigned long ingenic_tcu_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct ingenic_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct ingenic_tcu_clk_info *info = tcu_clk->info;
	unsigned int prescale;
	int ret;

	ret = regmap_read(tcu_clk->map, info->tcsr_reg, &prescale);
	WARN_ONCE(ret < 0, "Unable to read TCSR %i", tcu_clk->idx);

	prescale = (prescale & TCU_TCSR_PRESCALE_MASK) >> TCU_TCSR_PRESCALE_LSB;

	return parent_rate >> (prescale * 2);
}

static long ingenic_tcu_round_rate(struct clk_hw *hw, unsigned long req_rate,
		unsigned long *parent_rate)
{
	unsigned long rate = *parent_rate;
	unsigned int shift;

	if (req_rate > rate)
		return -EINVAL;

	for (shift = 0; shift < 10; shift += 2)
		if ((rate >> shift) <= req_rate)
			break;

	return rate >> shift;
}

static int ingenic_tcu_set_rate(struct clk_hw *hw, unsigned long req_rate,
		unsigned long parent_rate)
{
	struct ingenic_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct ingenic_tcu_clk_info *info = tcu_clk->info;
	struct regmap *map = tcu_clk->map;
	u8 prescale = (ffs(parent_rate / req_rate) / 2) << TCU_TCSR_PRESCALE_LSB;
	int ret;

	/*
	 * Our clock provider has the CLK_SET_RATE_GATE flag set, so we know
	 * that the clk is in unprepared state. To be able to access TCSR
	 * we must ungate the clock supply and we gate it again when done.
	 */

	regmap_write(map, TCU_REG_TSCR, BIT(info->gate_bit));

	ret = regmap_update_bits(map, info->tcsr_reg,
				TCU_TCSR_PRESCALE_MASK, prescale);
	WARN_ONCE(ret < 0, "Unable to update TCSR %i", tcu_clk->idx);

	regmap_write(map, TCU_REG_TSSR, BIT(info->gate_bit));

	return 0;
}

static const struct clk_ops ingenic_tcu_clk_ops = {
	.get_parent	= ingenic_tcu_get_parent,
	.set_parent	= ingenic_tcu_set_parent,

	.recalc_rate	= ingenic_tcu_recalc_rate,
	.round_rate	= ingenic_tcu_round_rate,
	.set_rate	= ingenic_tcu_set_rate,

	.enable		= ingenic_tcu_enable,
	.disable	= ingenic_tcu_disable,
	.is_enabled	= ingenic_tcu_is_enabled,
};

static const char * const ingenic_tcu_timer_parents[] = {
	"pclk",
	"rtc",
	"ext",
};

static const struct ingenic_tcu_clk_info ingenic_tcu_clk_info[] = {
#define DEF_TIMER(_name, _gate_bit, _tcsr)				\
	{								\
		.init_data = {						\
			.name = _name,					\
			.parent_names = ingenic_tcu_timer_parents,	\
			.num_parents = ARRAY_SIZE(ingenic_tcu_timer_parents),\
			.ops = &ingenic_tcu_clk_ops,			\
			.flags = CLK_SET_RATE_GATE | CLK_SET_PARENT_GATE,\
		},							\
		.gate_bit = _gate_bit,					\
		.tcsr_reg = _tcsr,					\
	}
	[JZ4740_CLK_TIMER0] = DEF_TIMER("timer0", 0, TCU_REG_TCSRc(0)),
	[JZ4740_CLK_TIMER1] = DEF_TIMER("timer1", 1, TCU_REG_TCSRc(1)),
	[JZ4740_CLK_TIMER2] = DEF_TIMER("timer2", 2, TCU_REG_TCSRc(2)),
	[JZ4740_CLK_TIMER3] = DEF_TIMER("timer3", 3, TCU_REG_TCSRc(3)),
	[JZ4740_CLK_TIMER4] = DEF_TIMER("timer4", 4, TCU_REG_TCSRc(4)),
	[JZ4740_CLK_TIMER5] = DEF_TIMER("timer5", 5, TCU_REG_TCSRc(5)),
	[JZ4740_CLK_TIMER6] = DEF_TIMER("timer6", 6, TCU_REG_TCSRc(6)),
	[JZ4740_CLK_TIMER7] = DEF_TIMER("timer7", 7, TCU_REG_TCSRc(7)),
	[JZ4740_CLK_WDT]    = DEF_TIMER("wdt",   16, TCU_REG_WDT_TCSR),
	[JZ4770_CLK_OST]    = DEF_TIMER("ost",   15, TCU_REG_OST_TCSR),
#undef DEF_TIMER
};

static int ingenic_tcu_register_clock(struct regmap *map, unsigned int idx,
		const struct ingenic_tcu_clk_info *info,
		struct clk_hw_onecell_data *clocks)
{
	struct ingenic_tcu_clk *tcu_clk;
	int err;

	tcu_clk = kzalloc(sizeof(*tcu_clk), GFP_KERNEL);
	if (!tcu_clk)
		return -ENOMEM;

	tcu_clk->hw.init = &info->init_data;
	tcu_clk->idx = idx;
	tcu_clk->info = info;
	tcu_clk->map = map;

	/* Set EXT as the default parent clock */
	ingenic_tcu_set_parent(&tcu_clk->hw, 2);

	ingenic_tcu_disable(&tcu_clk->hw);

	err = clk_hw_register(NULL, &tcu_clk->hw);
	if (err)
		goto err_free_tcu_clk;

	err = clk_hw_register_clkdev(&tcu_clk->hw, info->init_data.name, NULL);
	if (err)
		goto err_clk_unregister;

	clocks->hws[idx] = &tcu_clk->hw;
	return 0;

err_clk_unregister:
	clk_hw_unregister(&tcu_clk->hw);
err_free_tcu_clk:
	kfree(tcu_clk);
	return err;
}

static void __init ingenic_tcu_init(struct device_node *np,
		enum ingenic_version id)
{
	struct clk_hw_onecell_data *clocks;
	struct regmap *map;
	size_t i, nb_clks;
	int ret = -ENOMEM;

	if (id >= ID_JZ4770)
		nb_clks = (JZ4770_CLK_LAST - JZ4740_CLK_TIMER0) + 1;
	else
		nb_clks = (JZ4740_CLK_LAST - JZ4740_CLK_TIMER0) + 1;

	map = syscon_node_to_regmap(np->parent);
	if (IS_ERR(map)) {
		pr_err("%s: failed to map TCU registers\n", __func__);
		return;
	}

	clocks = kzalloc(sizeof(*clocks) +
					 sizeof(*clocks->hws) * nb_clks,
					 GFP_KERNEL);
	if (!clocks)
		return;

	clocks->num = nb_clks;

	for (i = 0; i < nb_clks; i++) {
		ret = ingenic_tcu_register_clock(map, i,
				&ingenic_tcu_clk_info[JZ4740_CLK_TIMER0 + i], clocks);
		if (ret) {
			pr_err("%s: cannot register clocks\n", __func__);
			goto err_unregister;
		}
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clocks);
	if (ret) {
		pr_err("%s: cannot add OF clock provider\n", __func__);
		goto err_unregister;
	}

	return;

err_unregister:
	for (i = 0; i < clocks->num; i++)
		if (clocks->hws[i])
			clk_hw_unregister(clocks->hws[i]);
	kfree(clocks);
}

static void __init jz4740_tcu_init(struct device_node *np)
{
	ingenic_tcu_init(np, ID_JZ4740);
}

static void __init jz4770_tcu_init(struct device_node *np)
{
	ingenic_tcu_init(np, ID_JZ4770);
}

static void __init jz4780_tcu_init(struct device_node *np)
{
	ingenic_tcu_init(np, ID_JZ4780);
}

/* We only probe via devicetree, no need for a platform driver */
CLK_OF_DECLARE(jz4740_tcu, "ingenic,jz4740-tcu-clocks", jz4740_tcu_init);
CLK_OF_DECLARE(jz4770_tcu, "ingenic,jz4770-tcu-clocks", jz4770_tcu_init);
CLK_OF_DECLARE(jz4780_tcu, "ingenic,jz4780-tcu-clocks", jz4780_tcu_init);

/*
 * JZ4740 SoC TCU clocks driver
 *
 * Copyright (c) 2015 Paul Cercueil <paul@crapouillou.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/ingenic,jz4740-tcu.h>

#define TCU_REG_TIMER_STOP		0x0C
#define TCU_REG_TIMER_STOP_SET		0x1C
#define TCU_REG_TIMER_STOP_CLEAR	0x2C

#define TCU_REG_TIMER_TCSR(x)		(0x3C + (x) * 0x10)

#define OST_REG_TCSR			0x0C
#define WDT_REG_TCSR			0x0C

#define TCSR_PARENT_CLOCK_MASK		0x07

#define TCSR_PRESCALE_LSB		3
#define TCSR_PRESCALE_MASK		0x38

enum jz4740_version {
	ID_JZ4740,
	ID_JZ4770,
	ID_JZ4780,
};

struct jz4740_tcu {
	struct device_node *np;
	void __iomem *base;
	void __iomem *ost_base;
	void __iomem *wdt_base;

	struct clk_onecell_data clocks;

	spinlock_t lock;
	struct regmap **tcsr_map;
};

struct jz4740_tcu_clk_info {
	struct clk_init_data init_data;
	u8 gate_bit;
	bool is_wdt, is_ost;
};

struct jz4740_tcu_clk {
	struct clk_hw hw;

	struct jz4740_tcu *tcu;
	const struct jz4740_tcu_clk_info *info;

	unsigned int idx;
};

#define to_tcu_clk(_hw) container_of(_hw, struct jz4740_tcu_clk, hw)

static int jz4740_tcu_enable(struct clk_hw *hw)
{
	struct jz4740_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct jz4740_tcu_clk_info *info = tcu_clk->info;
	struct jz4740_tcu *tcu = tcu_clk->tcu;

	writel(BIT(info->gate_bit), tcu->base + TCU_REG_TIMER_STOP_CLEAR);
	return 0;
}

static void jz4740_tcu_disable(struct clk_hw *hw)
{
	struct jz4740_tcu_clk *tcu_clk = to_tcu_clk(hw);
	struct jz4740_tcu *tcu = tcu_clk->tcu;
	const struct jz4740_tcu_clk_info *info = tcu_clk->info;

	writel(BIT(info->gate_bit), tcu->base + TCU_REG_TIMER_STOP_SET);
}

static int jz4740_tcu_is_enabled(struct clk_hw *hw)
{
	struct jz4740_tcu_clk *tcu_clk = to_tcu_clk(hw);
	struct jz4740_tcu *tcu = tcu_clk->tcu;
	const struct jz4740_tcu_clk_info *info = tcu_clk->info;

	return !(readl(tcu->base + TCU_REG_TIMER_STOP) & BIT(info->gate_bit));
}

static u8 jz4740_tcu_get_parent(struct clk_hw *hw)
{
	struct jz4740_tcu_clk *tcu_clk = to_tcu_clk(hw);
	struct regmap *tcsr = tcu_clk->tcu->tcsr_map[tcu_clk->idx];
	unsigned int val = 0;
	int ret;

	ret = regmap_read(tcsr, 0, &val);
	WARN_ONCE(ret < 0, "Unable to read TCSR %i", tcu_clk->idx);

	return (u8) ffs(val & TCSR_PARENT_CLOCK_MASK) - 1;
}

static int jz4740_tcu_set_parent(struct clk_hw *hw, u8 idx)
{
	struct jz4740_tcu_clk *tcu_clk = to_tcu_clk(hw);
	struct jz4740_tcu *tcu = tcu_clk->tcu;
	struct regmap *tcsr = tcu->tcsr_map[tcu_clk->idx];
	const struct jz4740_tcu_clk_info *info = tcu_clk->info;
	int ret;

	/*
	 * Our clock provider has the CLK_SET_PARENT_GATE flag set, so we know
	 * that the clk is in unprepared state. To be able to access TCSR
	 * we must ungate the clock supply and we gate it again when done.
	 */

	writel(BIT(info->gate_bit), tcu->base + TCU_REG_TIMER_STOP_CLEAR);

	ret = regmap_update_bits(tcsr, 0,
			TCSR_PARENT_CLOCK_MASK, BIT(idx));
	WARN_ONCE(ret < 0, "Unable to update TCSR %i", tcu_clk->idx);

	writel(BIT(info->gate_bit), tcu->base + TCU_REG_TIMER_STOP_SET);

	return 0;
}

static unsigned long jz4740_tcu_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct jz4740_tcu_clk *tcu_clk = to_tcu_clk(hw);
	struct regmap *tcsr = tcu_clk->tcu->tcsr_map[tcu_clk->idx];
	unsigned int prescale;
	int ret;

	ret = regmap_read(tcsr, 0, &prescale);
	WARN_ONCE(ret < 0, "Unable to read TCSR %i", tcu_clk->idx);

	prescale = (prescale & TCSR_PRESCALE_MASK) >> TCSR_PRESCALE_LSB;

	return parent_rate >> (prescale * 2);
}

static long jz4740_tcu_round_rate(struct clk_hw *hw, unsigned long req_rate,
		unsigned long *parent_rate)
{
	long rate = (long) *parent_rate;
	unsigned int shift;

	if (req_rate > rate)
		return -EINVAL;

	for (shift = 0; shift < 10; shift += 2)
		if ((rate >> shift) < req_rate)
			return rate >> shift;

	return rate >> 10;
}

static int jz4740_tcu_set_rate(struct clk_hw *hw, unsigned long req_rate,
		unsigned long parent_rate)
{
	struct jz4740_tcu_clk *tcu_clk = to_tcu_clk(hw);
	const struct jz4740_tcu_clk_info *info = tcu_clk->info;
	struct jz4740_tcu *tcu = tcu_clk->tcu;
	struct regmap *tcsr = tcu->tcsr_map[tcu_clk->idx];
	u8 prescale = (ffs(parent_rate / req_rate) / 2) << TCSR_PRESCALE_LSB;
	int ret;

	/*
	 * Our clock provider has the CLK_SET_RATE_GATE flag set, so we know
	 * that the clk is in unprepared state. To be able to access TCSR
	 * we must ungate the clock supply and we gate it again when done.
	 */

	writel(BIT(info->gate_bit), tcu->base + TCU_REG_TIMER_STOP_CLEAR);

	ret = regmap_update_bits(tcsr, 0, TCSR_PRESCALE_MASK, prescale);
	WARN_ONCE(ret < 0, "Unable to update TCSR %i", tcu_clk->idx);

	writel(BIT(info->gate_bit), tcu->base + TCU_REG_TIMER_STOP_SET);

	return 0;
}

static const struct clk_ops jz4740_tcu_clk_ops = {
	.get_parent	= jz4740_tcu_get_parent,
	.set_parent	= jz4740_tcu_set_parent,

	.recalc_rate	= jz4740_tcu_recalc_rate,
	.round_rate	= jz4740_tcu_round_rate,
	.set_rate	= jz4740_tcu_set_rate,

	.enable		= jz4740_tcu_enable,
	.disable	= jz4740_tcu_disable,
	.is_enabled	= jz4740_tcu_is_enabled,
};

static const char * const jz4740_tcu_timer_parents[] = {
	"pclk",
	"rtc",
	"ext",
};

static const struct jz4740_tcu_clk_info jz4740_tcu_clk_info[] = {
#define DEF_TIMER(_name, _gate_bit, _is_ost, _is_wdt)			\
	{								\
		.init_data = {						\
			.name = _name,					\
			.parent_names = jz4740_tcu_timer_parents,	\
			.num_parents = ARRAY_SIZE(jz4740_tcu_timer_parents),\
			.ops = &jz4740_tcu_clk_ops,			\
			.flags = CLK_SET_RATE_GATE | CLK_SET_PARENT_GATE, \
		},							\
		.gate_bit = _gate_bit,					\
		.is_ost = _is_ost,					\
		.is_wdt = _is_wdt,					\
	}
	[JZ4740_CLK_TIMER0] = DEF_TIMER("timer0", 0, false, false),
	[JZ4740_CLK_TIMER1] = DEF_TIMER("timer1", 1, false, false),
	[JZ4740_CLK_TIMER2] = DEF_TIMER("timer2", 2, false, false),
	[JZ4740_CLK_TIMER3] = DEF_TIMER("timer3", 3, false, false),
	[JZ4740_CLK_TIMER4] = DEF_TIMER("timer4", 4, false, false),
	[JZ4740_CLK_TIMER5] = DEF_TIMER("timer5", 5, false, false),
	[JZ4740_CLK_TIMER6] = DEF_TIMER("timer6", 6, false, false),
	[JZ4740_CLK_TIMER7] = DEF_TIMER("timer7", 7, false, false),
	[JZ4740_CLK_WDT]    = DEF_TIMER("wdt",   16, false, true),

	[JZ4770_CLK_OST]    = DEF_TIMER("ost",   15, true,  false),
#undef DEF_TIMER
};

static int jz4740_tcu_register_clock(struct jz4740_tcu *tcu, unsigned idx,
		const struct jz4740_tcu_clk_info *info)
{
	struct jz4740_tcu_clk *tcu_clk;
	struct clk *clk;
	int err;

	tcu_clk = kzalloc(sizeof(*tcu_clk), GFP_KERNEL);
	if (!tcu_clk)
		return -ENOMEM;

	tcu_clk->hw.init = &info->init_data;
	tcu_clk->idx = idx;
	tcu_clk->info = info;
	tcu_clk->tcu = tcu;

	/* Set EXT as the default parent clock */
	jz4740_tcu_set_parent(&tcu_clk->hw, 2);

	jz4740_tcu_disable(&tcu_clk->hw);

	clk = clk_register(NULL, &tcu_clk->hw);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		goto err_free_tcu_clk;
	}

	err = clk_register_clkdev(clk, info->init_data.name, NULL);
	if (err)
		goto err_clk_unregister;

	tcu->clocks.clks[idx] = clk;
	return 0;

err_clk_unregister:
	clk_unregister(clk);
err_free_tcu_clk:
	kfree(tcu_clk);
	return err;
}

static void __init ingenic_tcu_init(struct device_node *np,
		enum jz4740_version id)
{
	struct jz4740_tcu *tcu;
	size_t i, nb_clks;
	int ret = -ENOMEM;

	if (id >= ID_JZ4770) {
		nb_clks = (JZ4770_CLK_LAST - JZ4740_CLK_TIMER0) + 1;
	} else {
		nb_clks = (JZ4740_CLK_LAST - JZ4740_CLK_TIMER0) + 1;
	}

	tcu = kzalloc(sizeof(*tcu), GFP_KERNEL);
	if (!tcu) {
		pr_err("%s: cannot allocate memory\n", __func__);
		return;
	}

	tcu->tcsr_map = kzalloc(sizeof(*tcu->tcsr_map) * nb_clks, GFP_KERNEL);
	if (!tcu->tcsr_map)
		goto err_free_tcu;

	for (i = 0; i < nb_clks; i++) {
		struct device_node *tcsr_node;

		tcsr_node = of_parse_phandle(np, "tcsr", i);
		if (!tcsr_node)
			goto err_free_tcu_tcsr_map;

		tcu->tcsr_map[i] = syscon_node_to_regmap(tcsr_node);
		if (IS_ERR(tcu->tcsr_map[i])) {
			ret = PTR_ERR(tcu->tcsr_map[i]);
			goto err_free_tcu_tcsr_map;
		}
	}

	tcu->base = of_iomap(np, 0);
	if (!tcu->base) {
		pr_err("%s: failed to map TCU registers\n", __func__);
		goto err_free_tcu_tcsr_map;
	}

	tcu->wdt_base = of_iomap(np, 1);
	if (!tcu->wdt_base) {
		pr_err("%s: failed to map WDT registers\n", __func__);
		goto err_unmap_base;
	}

	if (id >= ID_JZ4770) {
		tcu->ost_base = of_iomap(np, 2);
		if (!tcu->ost_base) {
			pr_err("%s: failed to map OST registers\n", __func__);
			goto err_unmap_wdt_base;
		}
	}

	tcu->clocks.clk_num = nb_clks;
	tcu->clocks.clks = kzalloc(sizeof(struct clk *) * nb_clks, GFP_KERNEL);
	if (!tcu->clocks.clks) {
		pr_err("%s: cannot allocate memory\n", __func__);
		goto err_unmap_ost_base;
	}

	spin_lock_init(&tcu->lock);

	for (i = 0; i < nb_clks; i++) {
		ret = jz4740_tcu_register_clock(tcu, i,
				&jz4740_tcu_clk_info[JZ4740_CLK_TIMER0 + i]);
		if (ret) {
			pr_err("%s: cannot register clocks\n", __func__);
			goto err_unregister;
		}
	}

	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &tcu->clocks);
	if (ret) {
		pr_err("%s: cannot add OF clock provider\n", __func__);
		goto err_unregister;
	}

	return;

err_unregister:
	for (i = 0; i < tcu->clocks.clk_num; i++)
		if (tcu->clocks.clks[i])
			clk_unregister(tcu->clocks.clks[i]);
	kfree(tcu->clocks.clks);
err_unmap_ost_base:
	if (id >= ID_JZ4770)
		iounmap(tcu->ost_base);
err_unmap_wdt_base:
	iounmap(tcu->wdt_base);
err_unmap_base:
	iounmap(tcu->base);
err_free_tcu_tcsr_map:
	kfree(tcu->tcsr_map);
err_free_tcu:
	kfree(tcu);
}

static void __init jz4740_tcu_init(struct device_node *np)
{
	ingenic_tcu_init(np, ID_JZ4740);
}
CLK_OF_DECLARE(jz4740_tcu, "ingenic,jz4740-tcu-clocks", jz4740_tcu_init);

static void __init jz4770_tcu_init(struct device_node *np)
{
	ingenic_tcu_init(np, ID_JZ4770);
}
CLK_OF_DECLARE(jz4770_tcu, "ingenic,jz4770-tcu-clocks", jz4770_tcu_init);

static void __init jz4780_tcu_init(struct device_node *np)
{
	ingenic_tcu_init(np, ID_JZ4780);
}
CLK_OF_DECLARE(jz4780_tcu, "ingenic,jz4780-tcu-clocks", jz4780_tcu_init);

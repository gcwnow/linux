/*
 * Copyright (C) 2016 Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/compiler.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/ingenic-tcu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/sched_clock.h>
#include <linux/types.h>

#define TCU_OST_TCSR_MASK	0xc0
#define TCU_OST_TCSR_CNT_MD	BIT(15)

#define TCU_OST_CHANNEL		15

struct jz4770_ost {
#ifdef CONFIG_JZ4770_OST_CLOCKSOURCE
	struct clocksource cs;
#endif
	struct regmap *map;
	struct clk *timer_clk;
};

#ifdef CONFIG_JZ4770_OST_CLOCKSOURCE

static inline struct jz4770_ost *clocksource_to_ost(struct clocksource *cs)
{
	return container_of(cs, struct jz4770_ost, cs);
}

static u64 jz4770_ost_clocksource_read(struct clocksource *cs)
{
	struct regmap *map = clocksource_to_ost(cs)->map;
	u32 val1, val2;
	u64 count, recount;
	s64 diff;

	/*
	 * The buffering of the upper 32 bits of the timer prevents wrong
	 * results from the bottom 32 bits overflowing due to the timer ticking
	 * along. However, it does not prevent wrong results from simultaneous
	 * reads of the timer, which could reset the buffer mid-read.
	 * Since this kind of wrong read can happen only when the bottom bits
	 * overflow, there will be minutes between wrong reads, so if we read
	 * twice in succession, at least one of the reads will be correct.
	 */

	regmap_read(map, TCU_REG_OST_CNTL, &val1);
	regmap_read(map, TCU_REG_OST_CNTHBUF, &val2);
	count = (u64)val1 | (u64)val2 << 32;

	regmap_read(map, TCU_REG_OST_CNTL, &val1);
	regmap_read(map, TCU_REG_OST_CNTHBUF, &val2);
	recount = (u64)val1 | (u64)val2 << 32;

	/*
	 * A wrong read will produce a result that is 1<<32 too high: the bottom
	 * part from before overflow and the upper part from after overflow.
	 * Therefore, the lower value of the two reads is the correct value.
	 */

	diff = (s64)(recount - count);
	if (unlikely(diff < 0))
		count = recount;

	return count;
}

#endif /* CONFIG_JZ4770_OST_CLOCKSOURCE */

static struct jz4770_ost ost = {
#ifdef CONFIG_JZ4770_OST_CLOCKSOURCE
	.cs = {
		.name	= "jz4770-ost",
		.rating	= 320,
		.read	= jz4770_ost_clocksource_read,
		.mask	= CLOCKSOURCE_MASK(64),
		.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
	},
#endif
};

#ifdef CONFIG_JZ4770_OST_SCHED_CLOCK

static u64 notrace jz4770_ost_sched_read(void)
{
	u32 val;

	regmap_read(ost.map, TCU_REG_OST_CNTL, &val);

	return val;
}

#endif /* CONFIG_JZ4770_OST_SCHED_CLOCK */

static int __init jz4770_ost_init(struct device_node *np)
{
	unsigned long rate;
	int err;

	ost.map = syscon_node_to_regmap(np->parent);
	if (IS_ERR(ost.map)) {
		err = PTR_ERR(ost.map);
		goto err_end;
	}

	ost.timer_clk = of_clk_get_by_name(np, "ost");
	if (IS_ERR(ost.timer_clk)) {
		err = PTR_ERR(ost.timer_clk);
		pr_err("jz4770-ost: Failed to get clock\n");
		goto err_end;
	}

	err = clk_prepare_enable(ost.timer_clk);
	if (err)
		goto err_put_timer;

	regmap_write(ost.map, TCU_REG_OST_CNTL, 0);
	regmap_write(ost.map, TCU_REG_OST_CNTH, 0);

	/* Don't reset counter at compare value. */
	err = regmap_update_bits(ost.map, TCU_REG_OST_TCSR,
				TCU_OST_TCSR_MASK, TCU_OST_TCSR_CNT_MD);
	if (err)
		goto err_disable_timer;

	rate = clk_get_rate(ost.timer_clk);
	pr_info("jz4770-ost: OS Timer rate is %lu Hz\n", rate);

	regmap_write(ost.map, TCU_REG_TESR, BIT(TCU_OST_CHANNEL));

#ifdef CONFIG_JZ4770_OST_CLOCKSOURCE
	err = clocksource_register_hz(&ost.cs, rate);
	if (err)
		pr_err("jz4770-ost: clocksource registration failed: %d\n", err);
#endif

#ifdef CONFIG_JZ4770_OST_SCHED_CLOCK
	sched_clock_register(jz4770_ost_sched_read, 32, rate);
#endif

	return 0;

err_disable_timer:
	clk_disable_unprepare(ost.timer_clk);
err_put_timer:
	clk_put(ost.timer_clk);
err_end:
	pr_err("jz4770-ost: Init failed: %d\n", err);
	return err;
}

CLOCKSOURCE_OF_DECLARE(jz4770_ost, "ingenic,jz4770-ost", jz4770_ost_init);

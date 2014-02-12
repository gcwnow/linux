/*
 * Copyright (C) 2014 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/clocksource.h>
#include <linux/err.h>

#include "jz47xx-tcu.h"

static cycle_t jz4780_tcu_clocksource_read(struct clocksource *cs);

static struct jz47xx_tcu_channel_desc jz4780_tcu_channels[16] = {
	[0] = JZ47XX_TCU_CHANNEL(2, JZ47XX_TCU_CHANNEL_FIFO),
	[1] = JZ47XX_TCU_CHANNEL(2, 0),
	[2] = JZ47XX_TCU_CHANNEL(2, 0),
	[3] = JZ47XX_TCU_CHANNEL(2, JZ47XX_TCU_CHANNEL_FIFO),
	[4] = JZ47XX_TCU_CHANNEL(2, JZ47XX_TCU_CHANNEL_FIFO),
	[5] = JZ47XX_TCU_CHANNEL(1, JZ47XX_TCU_CHANNEL_FIFO),
	[6] = JZ47XX_TCU_CHANNEL(2, 0),
	[7] = JZ47XX_TCU_CHANNEL(2, 0),

	[15] = JZ47XX_TCU_CHANNEL(0, JZ47XX_TCU_CHANNEL_OST),
};

static struct jz47xx_tcu_desc jz4780_tcu_desc = {
	.channels = jz4780_tcu_channels,
	.num_channels = ARRAY_SIZE(jz4780_tcu_channels),
};

static struct {
	struct clocksource cs;
	struct jz47xx_tcu_channel *channel;
} jz4780_tcu_clocksource = {
	.cs = {
		.name	= "jz4780-tcu-clocksource",
		.rating	= 200,
		.read	= jz4780_tcu_clocksource_read,
		.mask	= CLOCKSOURCE_MASK(64),
		.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
	},
};

static cycle_t jz4780_tcu_clocksource_read(struct clocksource *cs)
{
	return jz47xx_tcu_read_channel_count(jz4780_tcu_clocksource.channel);
}

static void __init jz4780_tcu_init(struct device_node *np)
{
	struct jz47xx_tcu *tcu;
	unsigned rate;
	int err;

	tcu = jz47xx_tcu_init(&jz4780_tcu_desc, np);
	BUG_ON(IS_ERR(tcu));

	jz4780_tcu_clocksource.channel = jz47xx_tcu_req_channel(tcu, 15);
	BUG_ON(IS_ERR(jz4780_tcu_clocksource.channel));

	rate = jz47xx_tcu_set_channel_rate(jz4780_tcu_clocksource.channel,
					   JZ47XX_TCU_SRC_EXTAL, 1000000);
	pr_info("jz4780-tcu-clocksource: OST rate is %uHz\n", rate);
	BUG_ON(!rate);

	jz47xx_tcu_enable_channel(jz4780_tcu_clocksource.channel);

	err = clocksource_register_hz(&jz4780_tcu_clocksource.cs, rate);
	BUG_ON(err);

	/* For local clock events */
	err = jz47xx_tcu_setup_cevt(tcu, 5);
	BUG_ON(err);

	/* For tick broadcasts */
	err = jz47xx_tcu_setup_cevt(tcu, 6);
	BUG_ON(err);
}

CLOCKSOURCE_OF_DECLARE(jz4780_tcu, "ingenic,jz4780-tcu", jz4780_tcu_init);

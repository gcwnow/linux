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
#include <linux/sched_clock.h>

#include "jz47xx-tcu.h"

static struct jz47xx_tcu_channel_desc jz4780_tcu_channels[8] = {
	[0] = JZ47XX_TCU_CHANNEL(1, JZ47XX_TCU_CHANNEL_FIFO),
	[1] = JZ47XX_TCU_CHANNEL(1, 0),
	[2] = JZ47XX_TCU_CHANNEL(1, 0),
	[3] = JZ47XX_TCU_CHANNEL(1, JZ47XX_TCU_CHANNEL_FIFO),
	[4] = JZ47XX_TCU_CHANNEL(1, JZ47XX_TCU_CHANNEL_FIFO),
	[5] = JZ47XX_TCU_CHANNEL(0, JZ47XX_TCU_CHANNEL_FIFO),
	[6] = JZ47XX_TCU_CHANNEL(1, 0),
	[7] = JZ47XX_TCU_CHANNEL(1, 0),
};

static struct jz47xx_tcu_desc jz4780_tcu_desc = {
	.channels = jz4780_tcu_channels,
	.num_channels = ARRAY_SIZE(jz4780_tcu_channels),
};

static void __init jz4780_tcu_init(struct device_node *np)
{
	struct jz47xx_tcu *tcu;
	int err;

	tcu = jz47xx_tcu_init(&jz4780_tcu_desc, np);
	BUG_ON(IS_ERR(tcu));

	/* For local clock events */
	err = jz47xx_tcu_setup_cevt(tcu, 5);
	BUG_ON(err);

	/* For tick broadcasts */
	err = jz47xx_tcu_setup_cevt(tcu, 6);
	BUG_ON(err);
}

CLOCKSOURCE_OF_DECLARE(jz4780_tcu, "ingenic,jz4780-tcu", jz4780_tcu_init);

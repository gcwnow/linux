/*
 * Copyright (C) 2014 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _DRIVERS_CLOCKSOURCE_JZ47XX_TCU_H_
#define _DRIVERS_CLOCKSOURCE_JZ47XX_TCU_H_

#include <linux/of.h>
#include <linux/types.h>

struct jz47xx_tcu_channel_desc {
	u8 irq;
	u8 present: 1;
	unsigned flags;
#define JZ47XX_TCU_CHANNEL_FIFO	(1 << 0)
#define JZ47XX_TCU_CHANNEL_OST	(1 << 1)
};

#define JZ47XX_TCU_CHANNEL(_irq, _flags) {	\
	.irq = _irq,				\
	.present = 1,				\
	.flags = _flags,			\
}

struct jz47xx_tcu_desc {
	struct jz47xx_tcu_channel_desc *channels;
	unsigned num_channels;
};

struct jz47xx_tcu;
struct jz47xx_tcu_channel;

typedef void (jz47xx_tcu_irq_callback)(struct jz47xx_tcu_channel *channel,
				       void *data);

enum jz47xx_tcu_source {
	JZ47XX_TCU_SRC_PCLK,
	JZ47XX_TCU_SRC_RTCCLK,
	JZ47XX_TCU_SRC_EXTAL,
};

/**
 * jz47xx_tcu_init - initialise Ingenic jz47xx timer/counter unit (TCU)
 * desc: a description of the TCU in this SoC
 * np: the DT node representing the TCU
 */
extern struct jz47xx_tcu *jz47xx_tcu_init(const struct jz47xx_tcu_desc *desc,
					  struct device_node *np);

/**
 * jz47xx_tcu_req_channel - request a channel
 * tcu: the TCU reference returned by jz47xx_tcu_init
 * idx: the channel to request, or -1 for any available channel
 */
extern struct jz47xx_tcu_channel *jz47xx_tcu_req_channel(struct jz47xx_tcu *tcu,
							 int idx);

/**
 * jz47xx_tcu_release_channel - release a channel
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 */
extern void jz47xx_tcu_release_channel(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_setup_cevt - setup a clock event device using a channel
 * tcu: the TCU reference returned by jz47xx_tcu_init
 * idx: the channel to request, or -1 for any available channel
 */
extern int jz47xx_tcu_setup_cevt(struct jz47xx_tcu *tcu, int idx);

/**
 * jz47xx_tcu_set_channel_rate - set the count rate of a channel
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 * source: the source clock to be used
 * rate: the target count frequency in Hz
 *
 * Sets the channel up to use the specified source clock and count at a
 * frequency as close as possible to rate. Returns the actual frequency that
 * was set, or 0 on error.
 */
extern unsigned jz47xx_tcu_set_channel_rate(struct jz47xx_tcu_channel *channel,
					    enum jz47xx_tcu_source source,
					    unsigned rate);

/**
 * jz47xx_tcu_set_channel_rate - set the count rate of a channel
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 *
 * Returns the rate at which the channel is configured to count, or 0 on error.
 */
extern unsigned jz47xx_tcu_get_channel_rate(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_enable_channel - start a channel counting
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 */
extern void jz47xx_tcu_enable_channel(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_disable_channel - stop a channel counting
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 */
extern void jz47xx_tcu_disable_channel(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_mask_channel_full - mask a channels full interrupt
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 */
extern void jz47xx_tcu_mask_channel_full(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_unmask_channel_full - unmask a channels full interrupt
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 */
extern void jz47xx_tcu_unmask_channel_full(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_mask_channel_full - mask a channels half full interrupt
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 */
extern void jz47xx_tcu_mask_channel_half(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_unmask_channel_full - unmask a channels half full interrupt
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 */
extern void jz47xx_tcu_unmask_channel_half(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_read_channel_count - read the current channel count
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 */
extern u64 jz47xx_tcu_read_channel_count(struct jz47xx_tcu_channel *channel);

/**
 * jz47xx_tcu_set_channel_count - set the current channel count
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 * count: the count to set
 *
 * Returns zero on success, else -errno.
 */
extern int jz47xx_tcu_set_channel_count(struct jz47xx_tcu_channel *channel,
					u64 count);

/**
 * jz47xx_tcu_set_channel_full - set value at which the full interrupt triggers
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 * data: the value at which the full interrupt will trigger
 *
 * Returns zero on success, else -errno.
 */
extern int jz47xx_tcu_set_channel_full(struct jz47xx_tcu_channel *channel,
				       unsigned data);

/**
 * jz47xx_tcu_set_channel_half - set value at which the half interrupt triggers
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 * data: the value at which the half full interrupt will trigger
 *
 * Returns zero on success, else -errno.
 */
extern int jz47xx_tcu_set_channel_half(struct jz47xx_tcu_channel *channel,
				       unsigned data);

/**
 * jz47xx_tcu_set_channel_full_cb - set function to call upon a full interrupt
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 * cb: pointer to the function to be called
 * data: user-provided data to be passed to cb
 */
extern void jz47xx_tcu_set_channel_full_cb(struct jz47xx_tcu_channel *channel,
					   jz47xx_tcu_irq_callback *cb,
					   void *data);

/**
 * jz47xx_tcu_set_channel_half_cb - set function to call upon a half interrupt
 * channel: the TCU channel reference returned by jz47xx_tcu_req_channel
 * cb: pointer to the function to be called
 * data: user-provided data to be passed to cb
 */
extern void jz47xx_tcu_set_channel_half_cb(struct jz47xx_tcu_channel *channel,
					   jz47xx_tcu_irq_callback *cb,
					   void *data);

#endif /* _DRIVERS_CLOCKSOURCE_JZ47XX_TCU_H_ */

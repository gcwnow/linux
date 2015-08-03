/*
 * Copyright (C) 2014 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#include "jz47xx-tcu.h"

#define NUM_TCU_IRQS 3

enum jz47xx_tcu_reg {
	REG_TER		= 0x10,
	REG_TESR	= 0x14,
	REG_TECR	= 0x18,
	REG_TSR		= 0x1c,
	REG_TFR		= 0x20,
	REG_TFSR	= 0x24,
	REG_TFCR	= 0x28,
	REG_TSSR	= 0x2c,
	REG_TMR		= 0x30,
	REG_TMSR	= 0x34,
	REG_TMCR	= 0x38,
	REG_TSCR	= 0x3c,
	REG_TDFR0	= 0x40,
	REG_TDHR0	= 0x44,
	REG_TCNT0	= 0x48,
	REG_TCSR0	= 0x4c,
	REG_OSTDR	= 0xe0,
	REG_OSTCNTL	= 0xe4,
	REG_OSTCNTH	= 0xe8,
	REG_OSTCSR	= 0xec,
	REG_TSTR	= 0xf0,
	REG_TSTSR	= 0xf4,
	REG_TSTCR	= 0xf8,
	REG_OSTCNTHBUF	= 0xfc,
};

#define CHANNEL_STRIDE		0x10
#define REG_TDFRc(c)		(REG_TDFR0 + (c * CHANNEL_STRIDE))
#define REG_TDHRc(c)		(REG_TDHR0 + (c * CHANNEL_STRIDE))
#define REG_TCNTc(c)		(REG_TCNT0 + (c * CHANNEL_STRIDE))
#define REG_TCSRc(c)		(REG_TCSR0 + (c * CHANNEL_STRIDE))

#define TFR_HMASK_SHIFT		16

#define TMR_HMASK_SHIFT		16

#define TCSR_PRESCALE_SHIFT	3
#define TCSR_PRESCALE_MASK	(0x7 << TCSR_PRESCALE_SHIFT)
#define TCSR_EXT_EN		(1 << 2)
#define TCSR_RTC_EN		(1 << 1)
#define TCSR_PCK_EN		(1 << 0)
#define TCSR_SRC_MASK		(TCSR_EXT_EN | TCSR_RTC_EN | TCSR_PCK_EN)

#define OSTCSR_CNT_MD		(1 << 15)

struct jz47xx_tcu_channel {
	struct jz47xx_tcu *tcu;
	unsigned idx;
	jz47xx_tcu_irq_callback *full_cb, *half_cb;
	void *full_cb_data, *half_cb_data;
	unsigned stopped: 1;
	unsigned enabled: 1;
};

struct jz47xx_tcu_irq {
	struct jz47xx_tcu *tcu;
	unsigned long channel_map;
	unsigned channel;
	int virq;
};

struct jz47xx_tcu {
	const struct jz47xx_tcu_desc *desc;
	void __iomem *base;
	struct jz47xx_tcu_channel *channels;
	struct jz47xx_tcu_irq irqs[NUM_TCU_IRQS];
};

static inline u32 notrace tcu_readl(struct jz47xx_tcu *tcu, enum jz47xx_tcu_reg reg)
{
	return readl(tcu->base + reg);
}

static inline void tcu_writel(struct jz47xx_tcu *tcu, u32 val,
			      enum jz47xx_tcu_reg reg)
{
	writel(val, tcu->base + reg);
}

static enum jz47xx_tcu_reg tcu_csr(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;

	if (tcu->desc->channels[channel->idx].flags & JZ47XX_TCU_CHANNEL_OST)
		return REG_OSTCSR;

	return REG_TCSRc(channel->idx);
}

static void jz47xx_tcu_stop_channel(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << channel->idx, REG_TSSR);
	channel->stopped = true;
}

static void jz47xx_tcu_start_channel(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << channel->idx, REG_TSCR);
	channel->stopped = false;
}

void jz47xx_tcu_enable_channel(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << channel->idx, REG_TESR);
	channel->enabled = true;
}

void jz47xx_tcu_disable_channel(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << channel->idx, REG_TECR);
	channel->enabled = false;
}

void jz47xx_tcu_mask_channel_full(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << channel->idx, REG_TMSR);
}

void jz47xx_tcu_unmask_channel_full(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << channel->idx, REG_TMCR);
}

void jz47xx_tcu_mask_channel_half(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << (TMR_HMASK_SHIFT + channel->idx), REG_TMSR);
}

void jz47xx_tcu_unmask_channel_half(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << (TMR_HMASK_SHIFT + channel->idx), REG_TMCR);
}

static irqreturn_t jz47xx_tcu_single_channel_irq(int irq, void *dev_id)
{
	struct jz47xx_tcu_irq *tcu_irq = dev_id;
	struct jz47xx_tcu *tcu = tcu_irq->tcu;
	struct jz47xx_tcu_channel *chan;
	unsigned c = tcu_irq->channel;
	unsigned pending, ack;

	pending = tcu_readl(tcu, REG_TFR);
	chan = &tcu->channels[c];
	ack = 0;

	/* Callback for half full interrupt */
	if (pending & (1 << (TFR_HMASK_SHIFT + c))) {
		ack |= 1 << (TFR_HMASK_SHIFT + c);
		if (chan->half_cb)
			chan->half_cb(chan, chan->half_cb_data);
	}

	/* Callback for full interrupt */
	if (pending & (1 << c)) {
		ack |= 1 << c;
		if (chan->full_cb)
			chan->full_cb(chan, chan->full_cb_data);
	}

	/* Clear the match flags */
	tcu_writel(tcu, ack, REG_TFCR);

	return IRQ_HANDLED;
}

static irqreturn_t jz47xx_tcu_irq(int irq, void *dev_id)
{
	struct jz47xx_tcu_irq *tcu_irq = dev_id;
	struct jz47xx_tcu *tcu = tcu_irq->tcu;
	struct jz47xx_tcu_channel *chan;
	unsigned c, pending, ack;
	unsigned long pending_f, pending_h;

	pending = tcu_readl(tcu, REG_TFR);
	pending_f = pending & tcu_irq->channel_map;
	pending_h = (pending >> TFR_HMASK_SHIFT) & tcu_irq->channel_map;
	ack = 0;

	/* Callbacks for any pending half full interrupts */
	for_each_set_bit(c, &pending_h, tcu->desc->num_channels) {
		WARN_ON_ONCE(!tcu->desc->channels[c].present);

		chan = &tcu->channels[c];
		ack |= 1 << (TFR_HMASK_SHIFT + c);

		if (chan->half_cb)
			chan->half_cb(chan, chan->half_cb_data);
	}

	/* Callbacks for any pending full interrupts */
	for_each_set_bit(c, &pending_f, tcu->desc->num_channels) {
		WARN_ON_ONCE(!tcu->desc->channels[c].present);

		chan = &tcu->channels[c];
		ack |= 1 << c;

		if (chan->full_cb)
			chan->full_cb(chan, chan->full_cb_data);
	}

	/* Clear the match flags */
	tcu_writel(tcu, ack, REG_TFCR);

	return IRQ_HANDLED;
}

static struct irqaction jz47xx_tcu_irqaction[NUM_TCU_IRQS] = {
	{
		.flags = IRQF_TIMER,
		.name = "jz47xx-tcu-irq0",
	},
	{
		.flags = IRQF_TIMER,
		.name = "jz47xx-tcu-irq1",
	},
	{
		.flags = IRQF_TIMER,
		.name = "jz47xx-tcu-irq2",
	},
};

static int setup_tcu_irq(struct jz47xx_tcu *tcu, unsigned i)
{
	unsigned channel_count;

	channel_count = bitmap_weight(&tcu->irqs[i].channel_map,
				      tcu->desc->num_channels);
	if (channel_count == 1) {
		tcu->irqs[i].channel = find_first_bit(&tcu->irqs[i].channel_map,
						      tcu->desc->num_channels);

		jz47xx_tcu_irqaction[i].handler = jz47xx_tcu_single_channel_irq;
	} else {
		jz47xx_tcu_irqaction[i].handler = jz47xx_tcu_irq;
	}

	jz47xx_tcu_irqaction[i].dev_id = &tcu->irqs[i];

	return setup_irq(tcu->irqs[i].virq, &jz47xx_tcu_irqaction[i]);
}

struct jz47xx_tcu *jz47xx_tcu_init(const struct jz47xx_tcu_desc *desc,
				   struct device_node *np)
{
	struct jz47xx_tcu *tcu;
	unsigned i;
	int err;

	tcu = kzalloc(sizeof(*tcu), GFP_KERNEL);
	if (!tcu) {
		err = -ENOMEM;
		goto out;
	}

	tcu->channels = kzalloc(sizeof(*tcu->channels) * desc->num_channels,
				GFP_KERNEL);
	if (!tcu->channels) {
		err = -ENOMEM;
		goto out_free;
	}

	tcu->desc = desc;

	/* Map TCU registers */
	tcu->base = of_iomap(np, 0);
	if (!tcu->base) {
		err = -EINVAL;
		goto out_free;
	}

	/* Initialise all channels as stopped & calculate IRQ maps */
	for (i = 0; i < tcu->desc->num_channels; i++) {
		tcu->channels[i].tcu = tcu;
		tcu->channels[i].idx = i;
		jz47xx_tcu_stop_channel(&tcu->channels[i]);
		set_bit(i, &tcu->irqs[tcu->desc->channels[i].irq].channel_map);
	}

	/* Map IRQs */
	for (i = 0; i < NUM_TCU_IRQS; i++) {
		tcu->irqs[i].tcu = tcu;

		tcu->irqs[i].virq = irq_of_parse_and_map(np, i);
		if (!tcu->irqs[i].virq) {
			err = -EINVAL;
			goto out_irq_dispose;
		}

		err = setup_tcu_irq(tcu, i);
		if (err)
			goto out_irq_dispose;
	}

	return tcu;
out_irq_dispose:
	for (i = 0; i < ARRAY_SIZE(tcu->irqs); i++) {
		remove_irq(tcu->irqs[i].virq, &jz47xx_tcu_irqaction[i]);
		irq_dispose_mapping(tcu->irqs[i].virq);
	}
	iounmap(tcu->base);
out_free:
	kfree(tcu->channels);
	kfree(tcu);
out:
	return ERR_PTR(err);
}

struct jz47xx_tcu_channel *jz47xx_tcu_req_channel(struct jz47xx_tcu *tcu,
						  int idx)
{
	struct jz47xx_tcu_channel *channel;
	unsigned c;

	if (idx == -1) {
		for (c = 0; c < tcu->desc->num_channels; c++) {
			if (!tcu->desc->channels[c].present)
				continue;
			if (!tcu->channels[c].stopped)
				continue;
			idx = c;
			break;
		}
		if (idx == -1)
			return ERR_PTR(-ENODEV);
	}

	channel = &tcu->channels[idx];

	if (!channel->stopped)
		return ERR_PTR(-EBUSY);

	jz47xx_tcu_mask_channel_half(channel);
	jz47xx_tcu_mask_channel_full(channel);
	jz47xx_tcu_start_channel(channel);
	jz47xx_tcu_disable_channel(channel);

	if (tcu->desc->channels[channel->idx].flags & JZ47XX_TCU_CHANNEL_OST)
		tcu_writel(tcu, OSTCSR_CNT_MD, tcu_csr(channel));
	else
		tcu_writel(tcu, 0, tcu_csr(channel));

	return channel;
}

void jz47xx_tcu_release_channel(struct jz47xx_tcu_channel *channel)
{
	jz47xx_tcu_stop_channel(channel);
}

unsigned jz47xx_tcu_set_channel_rate(struct jz47xx_tcu_channel *channel,
				     enum jz47xx_tcu_source source,
				     unsigned rate)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	enum jz47xx_tcu_reg csr_reg;
	struct clk *src_clk;
	unsigned long src_rate;
	u32 csr, div, div_log2;

	/*
	 * Source & prescale can only be changed whilst the counter isn't
	 * counting.
	 */
	if (channel->enabled)
		return 0;

	csr_reg = tcu_csr(channel);
	csr = tcu_readl(tcu, csr_reg);
	csr &= ~(TCSR_PRESCALE_MASK | TCSR_SRC_MASK);

	switch (source) {
	case JZ47XX_TCU_SRC_PCLK:
		src_clk = clk_get(NULL, "pclk");
		csr |= TCSR_PCK_EN;
		break;
	case JZ47XX_TCU_SRC_RTCCLK:
		src_clk = clk_get(NULL, "rtc");
		csr |= TCSR_RTC_EN;
		break;
	case JZ47XX_TCU_SRC_EXTAL:
		src_clk = clk_get(NULL, "ext");
		csr |= TCSR_EXT_EN;
		break;
	default:
		return 0;
	}

	if (IS_ERR(src_clk)) {
		pr_info("%s failed to get src_clk: %ld\n", __func__,
			PTR_ERR(src_clk));
		return 0;
	}

	src_rate = clk_get_rate(src_clk);
	clk_put(src_clk);

	div = DIV_ROUND_CLOSEST(src_rate, rate);
	div_log2 = min(ilog2(div) & ~0x1, 10);
	div = 1 << div_log2;
	csr |= (div_log2 >> 1) << TCSR_PRESCALE_SHIFT;

	tcu_writel(tcu, csr, csr_reg);

	return src_rate / div;
}

unsigned jz47xx_tcu_get_channel_rate(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	struct clk *src_clk;
	unsigned long src_rate;
	u32 csr, prescale, div;

	csr = tcu_readl(tcu, tcu_csr(channel));

	if (csr & TCSR_PCK_EN)
		src_clk = clk_get(NULL, "pclk");
	else if (csr & TCSR_RTC_EN)
		src_clk = clk_get(NULL, "rtc");
	else if (csr & TCSR_EXT_EN)
		src_clk = clk_get(NULL, "ext");
	else
		return 0;

	if (IS_ERR(src_clk)) {
		pr_info("%s failed to get src_clk: %ld\n", __func__,
			PTR_ERR(src_clk));
		return 0;
	}

	src_rate = clk_get_rate(src_clk);
	clk_put(src_clk);

	prescale = (csr & TCSR_PRESCALE_MASK) >> TCSR_PRESCALE_SHIFT;
	div = 1 << (prescale * 2);

	return src_rate / div;
}

u64 notrace jz47xx_tcu_read_channel_count(struct jz47xx_tcu_channel *channel)
{
	struct jz47xx_tcu *tcu = channel->tcu;
	u64 count;

	if (tcu->desc->channels[channel->idx].flags & JZ47XX_TCU_CHANNEL_OST) {
		count = tcu_readl(tcu, REG_OSTCNTL);
		count |= (u64)tcu_readl(tcu, REG_OSTCNTHBUF) << 32;
	} else {
		count = tcu_readl(tcu, REG_TCNTc(channel->idx));
	}

	return count;
}

int jz47xx_tcu_set_channel_count(struct jz47xx_tcu_channel *channel,
				 u64 count)
{
	struct jz47xx_tcu *tcu = channel->tcu;

	if (tcu->desc->channels[channel->idx].flags & JZ47XX_TCU_CHANNEL_OST) {
		tcu_writel(tcu, count, REG_OSTCNTL);
		tcu_writel(tcu, count >> 32, REG_OSTCNTH);
		return 0;
	}

	if (count > 0xffff)
		return -EINVAL;

	tcu_writel(tcu, count, REG_TCNTc(channel->idx));
	return 0;
}

int jz47xx_tcu_set_channel_full(struct jz47xx_tcu_channel *channel,
				unsigned data)
{
	struct jz47xx_tcu *tcu = channel->tcu;

	if (tcu->desc->channels[channel->idx].flags & JZ47XX_TCU_CHANNEL_OST) {
		tcu_writel(tcu, data, REG_OSTDR);
		return 0;
	}

	if (data > 0xffff)
		return -EINVAL;

	tcu_writel(tcu, data, REG_TDFRc(channel->idx));
	return 0;
}

int jz47xx_tcu_set_channel_half(struct jz47xx_tcu_channel *channel,
				unsigned data)
{
	struct jz47xx_tcu *tcu = channel->tcu;

	if (tcu->desc->channels[channel->idx].flags & JZ47XX_TCU_CHANNEL_OST)
		return -EINVAL;

	if (data > 0xffff)
		return -EINVAL;

	tcu_writel(tcu, data, REG_TDFRc(channel->idx));
	return 0;
}

void jz47xx_tcu_set_channel_full_cb(struct jz47xx_tcu_channel *channel,
				    jz47xx_tcu_irq_callback *cb, void *data)
{
	channel->full_cb = cb;
	channel->full_cb_data = data;
}

void jz47xx_tcu_set_channel_half_cb(struct jz47xx_tcu_channel *channel,
				    jz47xx_tcu_irq_callback *cb, void *data)
{
	channel->half_cb = cb;
	channel->half_cb_data = data;
}

struct jz47xx_clock_event_device {
	struct clock_event_device cevt;
	struct jz47xx_tcu_channel *channel;
	char name[32];
};

#define jz47xx_cevt(_evt) \
	container_of(_evt, struct jz47xx_clock_event_device, cevt)

static void jz47xx_tcu_cevt_cb(struct jz47xx_tcu_channel *channel, void *data)
{
	struct clock_event_device *cevt = data;

	jz47xx_tcu_disable_channel(channel);

	if (cevt->event_handler)
		cevt->event_handler(cevt);
}

static void jz47xx_tcu_cevt_set_mode(enum clock_event_mode mode,
				     struct clock_event_device *evt)
{
	struct jz47xx_clock_event_device *jzcevt = jz47xx_cevt(evt);
	struct jz47xx_tcu_channel *channel = jzcevt->channel;

	switch (mode) {
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		jz47xx_tcu_disable_channel(channel);
		break;

	default:
		break;
	}
}

static int jz47xx_tcu_cevt_set_next(unsigned long next,
				    struct clock_event_device *evt)
{
	struct jz47xx_clock_event_device *jzcevt = jz47xx_cevt(evt);
	struct jz47xx_tcu_channel *channel = jzcevt->channel;

	jz47xx_tcu_set_channel_full(channel, next);
	jz47xx_tcu_set_channel_count(channel, 0);
	jz47xx_tcu_enable_channel(channel);

	return 0;
}

int jz47xx_tcu_setup_cevt(struct jz47xx_tcu *tcu, int idx)
{
	struct jz47xx_tcu_channel *channel;
	struct jz47xx_clock_event_device *jzcevt;
	unsigned rate;
	int err;

	channel = jz47xx_tcu_req_channel(tcu, idx);
	if (IS_ERR(channel)) {
		err = PTR_ERR(channel);
		goto err_out;
	}

	rate = jz47xx_tcu_set_channel_rate(channel, JZ47XX_TCU_SRC_EXTAL,
					   1000000);
	if (!rate) {
		err = -EINVAL;
		goto err_out_release;
	}

	jzcevt = kzalloc(sizeof(*jzcevt), GFP_KERNEL);
	if (!jzcevt) {
		err = -ENOMEM;
		goto err_out_release;
	}

	jzcevt->channel = channel;
	snprintf(jzcevt->name, sizeof(jzcevt->name), "jz478-tcu-chan%u",
		 channel->idx);

	jzcevt->cevt.cpumask = cpumask_of(smp_processor_id());
	jzcevt->cevt.features = CLOCK_EVT_FEAT_ONESHOT;
	jzcevt->cevt.name = jzcevt->name;
	jzcevt->cevt.rating = 200;
	jzcevt->cevt.set_mode = jz47xx_tcu_cevt_set_mode;
	jzcevt->cevt.set_next_event = jz47xx_tcu_cevt_set_next;

	jz47xx_tcu_set_channel_full_cb(channel, jz47xx_tcu_cevt_cb,
				       &jzcevt->cevt);
	jz47xx_tcu_unmask_channel_full(channel);
	clockevents_config_and_register(&jzcevt->cevt, rate, 10, (1 << 16) - 1);

	return 0;

err_out_release:
	jz47xx_tcu_release_channel(channel);
err_out:
	return err;
}

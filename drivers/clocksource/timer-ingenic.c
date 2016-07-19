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

#include <linux/clk/jz4740-tcu.h>

#define MIN_NUM_TCU_IRQS 2
#define MAX_NUM_TCU_IRQS 3

enum ingenic_soc_id {
	ID_JZ4740,
	ID_JZ4780,
};

enum ingenic_tcu_reg {
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
	REG_TSTR	= 0xf0,
	REG_TSTSR	= 0xf4,
	REG_TSTCR	= 0xf8,
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

struct ingenic_tcu;
struct ingenic_tcu_channel;

typedef void (ingenic_tcu_irq_callback)(struct ingenic_tcu_channel *channel,
				       void *data);

struct ingenic_tcu_channel {
	struct ingenic_tcu *tcu;
	unsigned idx;
	ingenic_tcu_irq_callback *full_cb, *half_cb;
	void *full_cb_data, *half_cb_data;
	unsigned stopped: 1;
	unsigned enabled: 1;
	struct clk *timer_clk, *counter_clk;
};

struct ingenic_tcu_irq {
	struct ingenic_tcu *tcu;
	unsigned long channel_map;
	unsigned channel;
	int virq;
};

struct ingenic_tcu {
	void __iomem *base;
	unsigned num_channels;
	struct ingenic_tcu_channel *channels;
	struct ingenic_tcu_irq irqs[MAX_NUM_TCU_IRQS];
};

static inline u32 tcu_readl(struct ingenic_tcu *tcu, enum ingenic_tcu_reg reg)
{
	return readl(tcu->base + reg);
}

static inline void tcu_writel(struct ingenic_tcu *tcu, u32 val,
			      enum ingenic_tcu_reg reg)
{
	writel(val, tcu->base + reg);
}

static void ingenic_tcu_stop_channel(struct ingenic_tcu_channel *channel)
{
	if (!channel->stopped)
		clk_disable(channel->counter_clk);
	channel->stopped = true;
}

static void ingenic_tcu_start_channel(struct ingenic_tcu_channel *channel)
{
	if (channel->stopped)
		clk_enable(channel->counter_clk);
	channel->stopped = false;
}

static void ingenic_tcu_enable_channel(struct ingenic_tcu_channel *channel)
{
	if (!channel->enabled)
		clk_enable(channel->timer_clk);
	channel->enabled = true;
}

static void ingenic_tcu_disable_channel(struct ingenic_tcu_channel *channel)
{
	if (channel->enabled)
		clk_disable(channel->timer_clk);
	channel->enabled = false;
}

static void ingenic_tcu_mask_channel_full(struct ingenic_tcu_channel *channel)
{
	struct ingenic_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << channel->idx, REG_TMSR);
}

static void ingenic_tcu_unmask_channel_full(struct ingenic_tcu_channel *channel)
{
	struct ingenic_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << channel->idx, REG_TMCR);
}

static void ingenic_tcu_mask_channel_half(struct ingenic_tcu_channel *channel)
{
	struct ingenic_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << (TMR_HMASK_SHIFT + channel->idx), REG_TMSR);
}

/*
static void ingenic_tcu_unmask_channel_half(struct ingenic_tcu_channel *channel)
{
	struct ingenic_tcu *tcu = channel->tcu;
	tcu_writel(tcu, 1 << (TMR_HMASK_SHIFT + channel->idx), REG_TMCR);
}
*/

static irqreturn_t ingenic_tcu_single_channel_irq(int irq, void *dev_id)
{
	struct ingenic_tcu_irq *tcu_irq = dev_id;
	struct ingenic_tcu *tcu = tcu_irq->tcu;
	struct ingenic_tcu_channel *chan;
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

static irqreturn_t ingenic_tcu_irq(int irq, void *dev_id)
{
	struct ingenic_tcu_irq *tcu_irq = dev_id;
	struct ingenic_tcu *tcu = tcu_irq->tcu;
	struct ingenic_tcu_channel *chan;
	unsigned c, pending, ack;
	unsigned long pending_f, pending_h;

	pending = tcu_readl(tcu, REG_TFR);
	pending_f = pending & tcu_irq->channel_map;
	pending_h = (pending >> TFR_HMASK_SHIFT) & tcu_irq->channel_map;
	ack = 0;

	/* Callbacks for any pending half full interrupts */
	for_each_set_bit(c, &pending_h, tcu->num_channels) {
		chan = &tcu->channels[c];
		ack |= 1 << (TFR_HMASK_SHIFT + c);

		if (chan->half_cb)
			chan->half_cb(chan, chan->half_cb_data);
	}

	/* Callbacks for any pending full interrupts */
	for_each_set_bit(c, &pending_f, tcu->num_channels) {
		chan = &tcu->channels[c];
		ack |= 1 << c;

		if (chan->full_cb)
			chan->full_cb(chan, chan->full_cb_data);
	}

	/* Clear the match flags */
	tcu_writel(tcu, ack, REG_TFCR);

	return IRQ_HANDLED;
}

static const char * const ingenic_tcu_irq_names[MAX_NUM_TCU_IRQS] = {
	"tcu-irq0", "tcu-irq1", "tcu-irq2",
};

static int setup_tcu_irq(struct ingenic_tcu *tcu, unsigned i)
{
	struct ingenic_tcu_irq *irq = &tcu->irqs[i];
	unsigned channel_count;
	irq_handler_t handler;

	channel_count = bitmap_weight(&irq->channel_map,
				      tcu->num_channels);
	if (channel_count == 1) {
		handler = ingenic_tcu_single_channel_irq;
		irq->channel = find_first_bit(&irq->channel_map,
				tcu->num_channels);
	} else {
		handler = ingenic_tcu_irq;
	}

	return request_irq(irq->virq, handler, IRQF_TIMER,
			ingenic_tcu_irq_names[i], irq);
}

static struct ingenic_tcu * __init ingenic_tcu_init_tcu(struct device_node *np)
{
	struct ingenic_tcu *tcu;
	unsigned i, irq;
	int err, num_channels, num_irqs;

	tcu = kzalloc(sizeof(*tcu), GFP_KERNEL);
	if (!tcu) {
		err = -ENOMEM;
		goto out;
	}

	num_channels = of_property_count_elems_of_size(np, "interrupts-map", 4);
	if (num_channels < 0) {
		err = -EINVAL;
		goto out;
	}

	num_irqs = of_property_count_elems_of_size(np, "interrupts", 4);
	if (num_irqs < MIN_NUM_TCU_IRQS || num_irqs > MAX_NUM_TCU_IRQS) {
		err = -EINVAL;
		goto out;
	}

	tcu->num_channels = num_channels;

	tcu->channels = kzalloc(sizeof(*tcu->channels) * tcu->num_channels,
				GFP_KERNEL);
	if (!tcu->channels) {
		err = -ENOMEM;
		goto out_free;
	}

	/* Map TCU registers */
	tcu->base = of_iomap(np, 0);
	if (!tcu->base) {
		err = -EINVAL;
		goto out_free;
	}

	/* Initialise all channels as stopped & calculate IRQ maps */
	for (i = 0; i < tcu->num_channels; i++) {
		struct clk *clk;
		char buf[16];

		snprintf(buf, sizeof(buf), "timer%u", i);
		clk = clk_get(NULL, buf);
		if (IS_ERR(clk)) {
			err = PTR_ERR(clk);
			goto out_clk_put;
		}

		tcu->channels[i].timer_clk = clk;
		clk_prepare(clk);

		snprintf(buf, sizeof(buf), "counter%u", i);
		clk = clk_get(NULL, buf);
		if (IS_ERR(clk)) {
			err = PTR_ERR(clk);
			goto out_clk_put;
		}

		tcu->channels[i].counter_clk = clk;
		clk_prepare(clk);

		tcu->channels[i].tcu = tcu;
		tcu->channels[i].idx = i;
		tcu->channels[i].stopped = true;

		err = of_property_read_u32_index(np, "interrupts-map", i, &irq);
		if (err)
			goto out_clk_put;

		set_bit(i, &tcu->irqs[irq].channel_map);
	}

	/* Map IRQs */
	for (i = 0; i < (unsigned) num_irqs; i++) {
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
		free_irq(tcu->irqs[i].virq, &tcu->irqs[i]);
		irq_dispose_mapping(tcu->irqs[i].virq);
	}
out_clk_put:
	for (i = 0; i < tcu->num_channels; i++) {
		if (tcu->channels[i].timer_clk)
			clk_put(tcu->channels[i].timer_clk);
		if (tcu->channels[i].counter_clk)
			clk_put(tcu->channels[i].counter_clk);
	}
	iounmap(tcu->base);
out_free:
	kfree(tcu->channels);
	kfree(tcu);
out:
	return ERR_PTR(err);
}

static struct ingenic_tcu_channel * __init ingenic_tcu_req_channel(
		struct ingenic_tcu *tcu, int idx)
{
	struct ingenic_tcu_channel *channel;
	unsigned c;

	if (idx == -1) {
		for (c = 0; c < tcu->num_channels; c++) {
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

	ingenic_tcu_enable_channel(channel);
	ingenic_tcu_mask_channel_half(channel);
	ingenic_tcu_mask_channel_full(channel);
	ingenic_tcu_start_channel(channel);
	ingenic_tcu_disable_channel(channel);
	jz4740_tcu_write_tcsr(channel->timer_clk, 0xffff, 0);

	return channel;
}

static void ingenic_tcu_release_channel(struct ingenic_tcu_channel *channel)
{
	ingenic_tcu_stop_channel(channel);
}

/*
static u64 ingenic_tcu_read_channel_count(struct ingenic_tcu_channel *channel)
{
	struct ingenic_tcu *tcu = channel->tcu;
	u64 count;

	count = tcu_readl(tcu, REG_TCNTc(channel->idx));

	return count;
}
*/

static int ingenic_tcu_set_channel_count(
		struct ingenic_tcu_channel *channel, u64 count)
{
	struct ingenic_tcu *tcu = channel->tcu;

	if (count > 0xffff)
		return -EINVAL;

	tcu_writel(tcu, count, REG_TCNTc(channel->idx));
	return 0;
}

static int ingenic_tcu_set_channel_full(
		struct ingenic_tcu_channel *channel, unsigned data)
{
	struct ingenic_tcu *tcu = channel->tcu;

	if (data > 0xffff)
		return -EINVAL;

	tcu_writel(tcu, data, REG_TDFRc(channel->idx));
	return 0;
}

/*
static int ingenic_tcu_set_channel_half(
		struct ingenic_tcu_channel *channel, unsigned data)
{
	struct ingenic_tcu *tcu = channel->tcu;

	if (data > 0xffff)
		return -EINVAL;

	tcu_writel(tcu, data, REG_TDFRc(channel->idx));
	return 0;
}
*/

static void ingenic_tcu_set_channel_full_cb(struct ingenic_tcu_channel *channel,
		ingenic_tcu_irq_callback *cb, void *data)
{
	channel->full_cb = cb;
	channel->full_cb_data = data;
}

/*
static void ingenic_tcu_set_channel_half_cb(struct ingenic_tcu_channel *channel,
		ingenic_tcu_irq_callback *cb, void *data)
{
	channel->half_cb = cb;
	channel->half_cb_data = data;
}
*/

struct ingenic_clock_event_device {
	struct clock_event_device cevt;
	struct ingenic_tcu_channel *channel;
	char name[32];
};

#define ingenic_cevt(_evt) \
	container_of(_evt, struct ingenic_clock_event_device, cevt)

static void ingenic_tcu_cevt_cb(struct ingenic_tcu_channel *channel, void *data)
{
	struct clock_event_device *cevt = data;

	ingenic_tcu_disable_channel(channel);

	if (cevt->event_handler)
		cevt->event_handler(cevt);
}

static int ingenic_tcu_cevt_set_state_shutdown(struct clock_event_device *evt)
{
	struct ingenic_clock_event_device *jzcevt = ingenic_cevt(evt);
	struct ingenic_tcu_channel *channel = jzcevt->channel;

        ingenic_tcu_disable_channel(channel);
        return 0;
}

static int ingenic_tcu_cevt_set_next(unsigned long next,
				    struct clock_event_device *evt)
{
	struct ingenic_clock_event_device *jzcevt = ingenic_cevt(evt);
	struct ingenic_tcu_channel *channel = jzcevt->channel;

	ingenic_tcu_set_channel_full(channel, next);
	ingenic_tcu_set_channel_count(channel, 0);
	ingenic_tcu_enable_channel(channel);

	return 0;
}

static int __init ingenic_tcu_setup_cevt(struct ingenic_tcu *tcu, int idx)
{
	struct ingenic_tcu_channel *channel;
	struct ingenic_clock_event_device *jzcevt;
	unsigned long rate;
	int err;

	channel = ingenic_tcu_req_channel(tcu, idx);
	if (IS_ERR(channel)) {
		err = PTR_ERR(channel);
		goto err_out;
	}

	rate = clk_get_rate(channel->timer_clk);
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
	snprintf(jzcevt->name, sizeof(jzcevt->name), "ingenic-tcu-chan%u",
		 channel->idx);

	jzcevt->cevt.cpumask = cpumask_of(smp_processor_id());
	jzcevt->cevt.features = CLOCK_EVT_FEAT_ONESHOT;
	jzcevt->cevt.name = jzcevt->name;
	jzcevt->cevt.rating = 200;
	jzcevt->cevt.set_state_shutdown = ingenic_tcu_cevt_set_state_shutdown;
	jzcevt->cevt.set_next_event = ingenic_tcu_cevt_set_next;

	ingenic_tcu_set_channel_full_cb(channel, ingenic_tcu_cevt_cb,
				       &jzcevt->cevt);
	ingenic_tcu_unmask_channel_full(channel);
	clockevents_config_and_register(&jzcevt->cevt, rate, 10, (1 << 16) - 1);

	return 0;

err_out_release:
	ingenic_tcu_release_channel(channel);
err_out:
	return err;
}

/*
static unsigned long ingenic_tcu_get_channel_rate(
		const struct ingenic_tcu_channel *chan)
{
	return clk_get_rate(chan->timer_clk);
}
*/

static void __init ingenic_tcu_init(struct device_node *np)
{
	struct ingenic_tcu *tcu;
	unsigned int i;
	int err;

	tcu = ingenic_tcu_init_tcu(np);
	BUG_ON(IS_ERR(tcu));

	for (i = 0; ; i++) {
		u32 value;

		err = of_property_read_u32_index(np, "timers", i, &value);
		if (err == -EOVERFLOW)
			break;
		BUG_ON(err);

		err = ingenic_tcu_setup_cevt(tcu, value);
		BUG_ON(err);
	}
}

CLOCKSOURCE_OF_DECLARE(jz4740_tcu, "ingenic,jz4740-tcu", ingenic_tcu_init);
CLOCKSOURCE_OF_DECLARE(jz4770_tcu, "ingenic,jz4770-tcu", ingenic_tcu_init);
CLOCKSOURCE_OF_DECLARE(jz4780_tcu, "ingenic,jz4780-tcu", ingenic_tcu_init);

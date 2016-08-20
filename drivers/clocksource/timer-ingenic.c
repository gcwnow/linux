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

struct ingenic_tcu;

struct ingenic_tcu_channel {
	struct ingenic_tcu *tcu;
	unsigned idx;
	unsigned stopped: 1;
	struct clk *timer_clk, *counter_clk;
};

struct ingenic_tcu {
	void __iomem *base;
	unsigned num_channels;
	struct ingenic_tcu_channel *channels;
	unsigned long requested;
};

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

static struct ingenic_tcu * __init ingenic_tcu_init_tcu(struct device_node *np,
		unsigned int num_channels)
{
	struct ingenic_tcu *tcu;
	unsigned i;
	int err;

	tcu = kzalloc(sizeof(*tcu), GFP_KERNEL);
	if (!tcu) {
		err = -ENOMEM;
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

	/* Initialise all channels as stopped */
	for (i = 0; i < tcu->num_channels; i++) {
		tcu->channels[i].tcu = tcu;
		tcu->channels[i].idx = i;
		tcu->channels[i].stopped = true;
	}

	return tcu;

out_free:
	kfree(tcu->channels);
	kfree(tcu);
out:
	return ERR_PTR(err);
}

static int __init ingenic_tcu_req_channel(struct ingenic_tcu_channel *channel)
{
	char buf[16];
	int err;

	if (test_and_set_bit(channel->idx, &channel->tcu->requested))
		return -EBUSY;

	snprintf(buf, sizeof(buf), "timer%u", channel->idx);
	channel->timer_clk = clk_get(NULL, buf);
	if (IS_ERR(channel->timer_clk)) {
		err = PTR_ERR(channel->timer_clk);
		goto out_release;
	}

	snprintf(buf, sizeof(buf), "counter%u", channel->idx);
	channel->counter_clk = clk_get(NULL, buf);
	if (IS_ERR(channel->counter_clk)) {
		err = PTR_ERR(channel->counter_clk);
		goto out_timer_clk_put;
	}

	err = clk_prepare_enable(channel->timer_clk);
	if (err)
		goto out_counter_clk_put;

	err = clk_prepare(channel->counter_clk);
	if (err)
		goto out_timer_clk_disable_unprepare;

	jz4740_tcu_write_tcsr(channel->timer_clk, 0xffff, 0);

	return 0;

out_timer_clk_disable_unprepare:
	clk_disable_unprepare(channel->timer_clk);
out_counter_clk_put:
	clk_put(channel->counter_clk);
out_timer_clk_put:
	clk_put(channel->timer_clk);
out_release:
	clear_bit(channel->idx, &channel->tcu->requested);
	return err;
}

static void __init ingenic_tcu_free_channel(struct ingenic_tcu_channel *channel)
{
	clk_unprepare(channel->counter_clk);
	clk_disable_unprepare(channel->timer_clk);
	clk_put(channel->counter_clk);
	clk_put(channel->timer_clk);
	clear_bit(channel->idx, &channel->tcu->requested);
}

struct ingenic_clock_event_device {
	struct clock_event_device cevt;
	struct ingenic_tcu_channel *channel;
	char name[32];
};

#define ingenic_cevt(_evt) \
	container_of(_evt, struct ingenic_clock_event_device, cevt)

static int ingenic_tcu_cevt_set_state_shutdown(struct clock_event_device *evt)
{
	struct ingenic_clock_event_device *jzcevt = ingenic_cevt(evt);
	struct ingenic_tcu_channel *channel = jzcevt->channel;

	ingenic_tcu_stop_channel(channel);
	return 0;
}

static int ingenic_tcu_cevt_set_next(unsigned long next,
		struct clock_event_device *evt)
{
	struct ingenic_clock_event_device *jzcevt = ingenic_cevt(evt);
	struct ingenic_tcu_channel *channel = jzcevt->channel;
	struct ingenic_tcu *tcu = channel->tcu;
	unsigned idx = channel->idx;

	if (next > 0xffff)
		return -EINVAL;

	writel((unsigned int) next, tcu->base + REG_TDFRc(idx));
	writel(0, tcu->base + REG_TCNTc(idx));

	ingenic_tcu_start_channel(channel);

	return 0;
}

static const char * const ingenic_tcu_timer_names[] = {
	"TCU0", "TCU1", "TCU2", "TCU3", "TCU4", "TCU5", "TCU6", "TCU7",
};

static irqreturn_t ingenic_tcu_cevt_cb(int irq, void *dev_id)
{
	struct clock_event_device *cevt = dev_id;
	struct ingenic_clock_event_device *jzcevt = ingenic_cevt(cevt);
	struct ingenic_tcu_channel *channel = jzcevt->channel;

	ingenic_tcu_stop_channel(channel);

	if (cevt->event_handler)
		cevt->event_handler(cevt);

	return IRQ_HANDLED;
}

static int __init ingenic_tcu_setup_cevt(struct device_node *np,
		struct ingenic_tcu *tcu, unsigned int idx)
{
	struct ingenic_tcu_channel *channel = &tcu->channels[idx];
	struct ingenic_clock_event_device *jzcevt;
	unsigned long rate;
	int err, virq;

	err = ingenic_tcu_req_channel(channel);
	if (err)
		return err;

	rate = clk_get_rate(channel->timer_clk);
	if (!rate) {
		err = -EINVAL;
		goto err_out_free_channel;
	}

	jzcevt = kzalloc(sizeof(*jzcevt), GFP_KERNEL);
	if (!jzcevt) {
		err = -ENOMEM;
		goto err_out_free_channel;
	}

	virq = irq_of_parse_and_map(np, idx);
	if (!virq) {
		err = -EINVAL;
		goto err_out_kfree_jzcevt;
	}

	err = request_irq(virq, ingenic_tcu_cevt_cb, IRQF_TIMER,
			ingenic_tcu_timer_names[idx], &jzcevt->cevt);
	if (err)
		goto err_out_irq_dispose_mapping;

	jzcevt->channel = channel;
	snprintf(jzcevt->name, sizeof(jzcevt->name), "ingenic-tcu-chan%u",
		 channel->idx);

	jzcevt->cevt.cpumask = cpumask_of(smp_processor_id());
	jzcevt->cevt.features = CLOCK_EVT_FEAT_ONESHOT;
	jzcevt->cevt.name = jzcevt->name;
	jzcevt->cevt.rating = 200;
	jzcevt->cevt.set_state_shutdown = ingenic_tcu_cevt_set_state_shutdown;
	jzcevt->cevt.set_next_event = ingenic_tcu_cevt_set_next;

	clockevents_config_and_register(&jzcevt->cevt, rate, 10, (1 << 16) - 1);

	return 0;

err_out_irq_dispose_mapping:
	irq_dispose_mapping(virq);
err_out_kfree_jzcevt:
	kfree(jzcevt);
err_out_free_channel:
	ingenic_tcu_free_channel(channel);
	return err;
}

static void __init ingenic_tcu_init(struct device_node *np)
{
	struct ingenic_tcu *tcu;
	unsigned i;
	int err, num_timers, num_channels;

	num_timers = of_property_count_elems_of_size(np, "timers", 4);
	BUG_ON(num_timers < 0);

	num_channels = of_property_count_elems_of_size(np, "interrupts", 4);
	BUG_ON(num_channels < 0 || num_channels > 8);

	tcu = ingenic_tcu_init_tcu(np, num_channels);
	BUG_ON(IS_ERR(tcu));

	for (i = 0; i < (unsigned) num_timers; i++) {
		u32 timer;

		err = of_property_read_u32_index(np, "timers", i, &timer);
		BUG_ON(err);

		err = ingenic_tcu_setup_cevt(np, tcu, timer);
		BUG_ON(err);
	}
}

CLOCKSOURCE_OF_DECLARE(jz4740_tcu, "ingenic,jz4740-tcu", ingenic_tcu_init);
CLOCKSOURCE_OF_DECLARE(jz4770_tcu, "ingenic,jz4770-tcu", ingenic_tcu_init);
CLOCKSOURCE_OF_DECLARE(jz4780_tcu, "ingenic,jz4780-tcu", ingenic_tcu_init);

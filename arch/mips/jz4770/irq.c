/*
 * linux/arch/mips/jz4770/irq.c
 *
 * JZ4770 interrupt routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/ioport.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/bitops.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/mipsregs.h>

#include <asm/mach-jz4770/jz4770dmac.h>

#include "gpio.h"
#include "intc.h"

/*
 * DMA irq type
 */
static void enable_dma_irq(struct irq_data *data)
{
	unsigned int irq = data->irq;
	unsigned int intc_irq;

	if ( irq < (IRQ_DMA_0 + DMA_IRQ_NUM / 2) ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_DMAC0;
	else if ( irq < (IRQ_DMA_0 + DMA_IRQ_NUM) ) 	/* DMAC Group 1 irq */
		intc_irq = IRQ_DMAC1;
	else {
		printk("%s, unexpected dma irq #%d\n", __FILE__, irq);
		return;
	}
	__intc_unmask_irq(intc_irq);
	__dmac_channel_enable_irq(irq - IRQ_DMA_0);
}

static void disable_dma_irq(struct irq_data *data)
{
	int chan = data->irq - IRQ_DMA_0;
	__dmac_disable_channel(chan);
	__dmac_channel_disable_irq(chan);
}

static void mask_and_ack_dma_irq(struct irq_data *data)
{
	unsigned int irq = data->irq;
	unsigned int intc_irq;

	disable_dma_irq(data);

	if ( irq < (IRQ_DMA_0 + HALF_DMA_NUM) ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_DMAC0;
	else if ( irq < (IRQ_DMA_0 + MAX_DMA_NUM) ) 	/* DMAC Group 1 irq */
		intc_irq = IRQ_DMAC1;
	else {
		printk("%s, unexpected dma irq #%d\n", __FILE__, irq);
		return ;
	}
	__intc_ack_irq(intc_irq);
	//__dmac_channel_ack_irq(irq-IRQ_DMA_0); /* needed?? add 20080506, Wolfgang */
	//__dmac_channel_disable_irq(irq - IRQ_DMA_0);
}

static unsigned int startup_dma_irq(struct irq_data *data)
{
	enable_dma_irq(data);
	return 0;
}

static void shutdown_dma_irq(struct irq_data *data)
{
	disable_dma_irq(data);
}

static struct irq_chip dma_irq_type = {
	.name = "DMA",
	.irq_startup = startup_dma_irq,
	.irq_shutdown = shutdown_dma_irq,
	.irq_unmask = enable_dma_irq,
	.irq_mask = disable_dma_irq,
	.irq_ack = mask_and_ack_dma_irq,
};

//----------------------------------------------------------------------

static irqreturn_t jz4770_dma_cascade(int irq, void *data)
{
	return generic_handle_irq(__dmac_get_irq() + IRQ_DMA_0);
}

static struct irqaction jz4770_dma_cascade_action = {
	.name = "JZ4770 DMA cascade interrupt",
	.handler = jz4770_dma_cascade,
};

void __init arch_init_irq(void)
{
	int i;

	irqchip_init();

	/* Set up DMAC irq. */
	for (i = IRQ_DMA_0; i < IRQ_DMA_0 + DMA_IRQ_NUM; i++) {
		disable_dma_irq(&irq_desc[i].irq_data);
		irq_set_chip_and_handler(i, &dma_irq_type, handle_level_irq);
	}

	setup_irq(IRQ_DMAC0, &jz4770_dma_cascade_action);
	setup_irq(IRQ_DMAC1, &jz4770_dma_cascade_action);
}

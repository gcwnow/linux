/*
 *  linux/include/asm-mips/mach-jz4770/dma.h
 *
 *  JZ4770 DMA definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4770_DMA_H__
#define __ASM_JZ4770_DMA_H__

#include <linux/interrupt.h>
#include <asm/io.h>			/* need byte IO */
#include <linux/spinlock.h>		/* And spinlocks */
#include <linux/delay.h>


/*
 * Descriptor structure for JZ4770 DMA engine
 * Note: this structure must always be aligned to a 16-bytes boundary.
 */

/* new descriptor 8-word */
struct jz_dma_desc_8word {
	volatile u32 dcmd;	/* DCMD value for the current transfer */
	volatile u32 dsadr;	/* DSAR value for the current transfer */
	volatile u32 dtadr;	/* DTAR value for the current transfer */
	volatile u32 ddadr;	/* Points to the next descriptor + transfer count */
	volatile u32 dstrd;     /* DMA source and target stride address */
	volatile u32 dreqt;     /* DMA request type for current transfer */
	volatile u32 reserved0;	/* Reserved */
	volatile u32 reserved1;	/* Reserved */
};

/* DMA Device ID's follow */
enum {
	DMA_ID_AUTO = 0,		/* Auto-request */
//	DMA_ID_TSSI_RX,		/* TSSI receive fifo full request */
	DMA_ID_UART3_TX,	/* UART3 transmit-fifo-empty request */
	DMA_ID_UART3_RX,	/* UART3 receve-fifo-full request */
	DMA_ID_UART2_TX,	/* UART2 transmit-fifo-empty request */
	DMA_ID_UART2_RX,	/* UART2 receve-fifo-full request */
	DMA_ID_UART1_TX,	/* UART1 transmit-fifo-empty request */
	DMA_ID_UART1_RX,	/* UART1 receve-fifo-full request */
	DMA_ID_UART0_TX,	/* UART0 transmit-fifo-empty request */
	DMA_ID_UART0_RX,	/* UART0 receve-fifo-full request */
	DMA_ID_SSI0_TX,		/* SSI0 transmit-fifo-full request */
	DMA_ID_SSI0_RX,		/* SSI0 receive-fifo-empty request */
	DMA_ID_AIC_TX,		/* AIC transmit-fifo-full request */
	DMA_ID_AIC_RX,		/* AIC receive-fifo-empty request */
	DMA_ID_MSC0,
	DMA_ID_TCU_OVERFLOW,	/* TCU channel n overflow interrupt */
	DMA_ID_SADC,		/* SADC transfer request */
	DMA_ID_MSC1,
	DMA_ID_MSC2,
	DMA_ID_SSI1_TX,		/* SSI1 transmit-fifo-full request */
	DMA_ID_SSI1_RX,		/* SSI1 receive-fifo-empty request */
	DMA_ID_PCM0_TX,
	DMA_ID_PCM0_RX,
	DMA_ID_PCM1_TX,
	DMA_ID_PCM1_RX,
	DMA_ID_RAW_SET,
	DMA_ID_I2C0_RX,
	DMA_ID_I2C0_TX,
	DMA_ID_I2C1_RX,
	DMA_ID_I2C1_TX,
	DMA_ID_I2C2_RX,
	DMA_ID_I2C2_TX,
	DMA_ID_MAX
};

/* DMA modes, simulated by sw */
#define DMA_MODE_READ	0x0  /* I/O to memory, no autoinit, increment, single mode */
#define DMA_MODE_WRITE	0x1  /* memory to I/O, no autoinit, increment, single mode */
#define DMA_AUTOINIT	0x2
#define DMA_MODE_MASK	0x3


extern int jz_request_dma(int dev_id,
			  const char *dev_str,
			  irqreturn_t (*irqhandler)(int, void *),
			  unsigned long irqflags,
			  void *irq_dev_id);
extern void jz_stop_dma(unsigned int chan);
extern void jz_free_dma(unsigned int dmanr);

extern void enable_dma(unsigned int dmanr);
extern void disable_dma(unsigned int dmanr);
extern spinlock_t  dma_spin_lock;

static __inline__ unsigned long claim_dma_lock(void)
{
	unsigned long flags;
	spin_lock_irqsave(&dma_spin_lock, flags);
	return flags;
}

static __inline__ void release_dma_lock(unsigned long flags)
{
	spin_unlock_irqrestore(&dma_spin_lock, flags);
}

#endif  /* __ASM_JZ4770_DMA_H__ */

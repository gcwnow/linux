/*
 * linux/arch/mips/jz4770/dma.c
 *
 * Support functions for the JZ4770 internal DMA channels.
 * No-descriptor transfer only.
 * Descriptor transfer should also call jz_request_dma() to get a free
 * channel and call jz_free_dma() to free the channel. And driver should
 * build the DMA descriptor and setup the DMA channel by itself.
 *
 * Copyright (C) 2006 - 2008 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/soundcard.h>

#include <asm/addrspace.h>

#include <asm/mach-jz4770/dma.h>
#include <asm/mach-jz4770/jz4770aic.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770pcm.h>
#include <asm/mach-jz4770/jz4770uart.h>


/*
 * A note on resource allocation:
 *
 * All drivers needing DMA channels, should allocate and release them
 * through the public routines `jz_request_dma()' and `jz_free_dma()'.
 *
 * In order to avoid problems, all processes should allocate resources in
 * the same sequence and release them in the reverse order.
 *
 * So, when allocating DMAs and IRQs, first allocate the DMA, then the IRQ.
 * When releasing them, first release the IRQ, then release the DMA. The
 * main reason for this order is that, if you are requesting the DMA buffer
 * done interrupt, you won't know the irq number until the DMA channel is
 * returned from jz_request_dma().
 */

struct jz_dma_chan jz_dma_table[MAX_DMA_NUM] = {
	{ dev_id: DMA_ID_MSC0, },	/* DMAC0 channel 0, reserved for MSC0 */
	{ dev_id: -1, },		/* DMAC0 channel 1 */
	{ dev_id: -1, },		/* DMAC0 channel 2 */
	{ dev_id: -1, },		/* DMAC0 channel 3 */
	{ dev_id: -1, },		/* DMAC0 channel 4 */
	{ dev_id: -1, },			/* DMAC0 channel 5 --- unavailable */

	/* To avoid bug, reserved channel 6 & 7 for AIC_TX & AIC_RX */
	{ dev_id: DMA_ID_AIC_TX, },	/* DMAC1 channel 0 */
	{ dev_id: DMA_ID_AIC_RX, },	/* DMAC1 channel 1 */
	{ dev_id: DMA_ID_MSC1, },	/* DMAC1 channel 2, reserved for MSC1 */
	{ dev_id: -1, },		/* DMAC1 channel 3 */
	{ dev_id: -1, },		/* DMAC0 channel 4 */
	{ dev_id: -1, },			/* DMAC0 channel 5 --- unavailable */
};

// Device FIFO addresses and default DMA modes
static const struct {
	unsigned int fifo_addr;
	unsigned int dma_mode;
	unsigned int dma_source;
} dma_dev_table[DMA_ID_MAX] = {
	[DMA_ID_AUTO] = {0, DMA_AUTOINIT, DMAC_DRSR_RS_AUTO},
//	{CPHYSADDR(TSSI_FIFO), DMA_32BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_TSSIIN},
	[DMA_ID_UART3_TX] = {CPHYSADDR(UART3_TDR), DMA_8BIT_TX_CMD | DMA_MODE_WRITE, DMAC_DRSR_RS_UART3OUT},
	[DMA_ID_UART3_RX] = {CPHYSADDR(UART3_RDR), DMA_8BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_UART3IN},
	[DMA_ID_UART2_TX] = {CPHYSADDR(UART2_TDR), DMA_8BIT_TX_CMD | DMA_MODE_WRITE, DMAC_DRSR_RS_UART2OUT},
	[DMA_ID_UART2_RX] = {CPHYSADDR(UART2_RDR), DMA_8BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_UART2IN},
	[DMA_ID_UART1_TX] = {CPHYSADDR(UART1_TDR), DMA_8BIT_TX_CMD | DMA_MODE_WRITE, DMAC_DRSR_RS_UART1OUT},
	[DMA_ID_UART1_RX] = {CPHYSADDR(UART1_RDR), DMA_8BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_UART1IN},
	[DMA_ID_UART0_TX] = {CPHYSADDR(UART0_TDR), DMA_8BIT_TX_CMD | DMA_MODE_WRITE, DMAC_DRSR_RS_UART0OUT},
	[DMA_ID_UART0_RX] = {CPHYSADDR(UART0_RDR), DMA_8BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_UART0IN},
	//[DMA_ID_SSI0_TX] = {CPHYSADDR(SSI_DR(0)), DMA_32BIT_TX_CMD | DMA_MODE_WRITE, DMAC_DRSR_RS_SSI0OUT},
	//[DMA_ID_SSI0_RX] = {CPHYSADDR(SSI_DR(0)), DMA_32BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_SSI0IN},
	[DMA_ID_AIC_TX] = {CPHYSADDR(AIC_DR), DMA_AIC_TX_CMD_UNPACK | DMA_MODE_WRITE, DMAC_DRSR_RS_AICOUT},
	[DMA_ID_AIC_RX] = {CPHYSADDR(AIC_DR), DMA_32BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_AICIN},
	[DMA_ID_MSC0] = {0, 0, 0},
	[DMA_ID_TCU_OVERFLOW] = {0, DMA_AUTOINIT, DMAC_DRSR_RS_TCU},
	//[DMA_ID_SADC] = {CPHYSADDR(SADC_ADTCH), DMA_32BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_SADC},/* Touch Screen Data Register */
	[DMA_ID_SADC] = { 0, 0, 0 },
	[DMA_ID_MSC1] = {0, 0, 0},
	[DMA_ID_MSC2] = {0, 0, 0},
	//[DMA_ID_SSI1_TX] = {CPHYSADDR(SSI_DR(1)), DMA_32BIT_TX_CMD | DMA_MODE_WRITE, DMAC_DRSR_RS_SSI1OUT},
	//[DMA_ID_SSI1_RX] = {CPHYSADDR(SSI_DR(1)), DMA_32BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_SSI1IN},
	[DMA_ID_PCM0_TX] = {CPHYSADDR(PCM_PDP(0)), DMA_16BIT_TX_CMD | DMA_MODE_WRITE, DMAC_DRSR_RS_PM0OUT},
	[DMA_ID_PCM0_RX] = {CPHYSADDR(PCM_PDP(0)), DMA_16BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_PM0IN},
	[DMA_ID_PCM1_TX] = {CPHYSADDR(PCM_PDP(1)), DMA_16BIT_TX_CMD | DMA_MODE_WRITE, DMAC_DRSR_RS_PM1OUT},
	[DMA_ID_PCM1_RX] = {CPHYSADDR(PCM_PDP(1)), DMA_16BIT_RX_CMD | DMA_MODE_READ, DMAC_DRSR_RS_PM1IN},
	[DMA_ID_I2C0_RX] = { 0, 0, 0 },
	[DMA_ID_I2C1_RX] = { 0, 0, 0 },
	[DMA_ID_I2C2_RX] = { 0, 0, 0 },
	[DMA_ID_I2C0_TX] = { 0, 0, 0 },
	[DMA_ID_I2C1_TX] = { 0, 0, 0 },
	[DMA_ID_I2C2_TX] = { 0, 0, 0 },
};


int jz_dma_read_proc(char *buf, char **start, off_t fpos,
			 int length, int *eof, void *data)
{
	int i, len = 0;
	struct jz_dma_chan *chan;

	for (i = 0; i < MAX_DMA_NUM; i++) {
		if ((chan = get_dma_chan(i)) != NULL) {
			len += sprintf(buf + len, "%2d: %s\n",
				       i, chan->dev_str);
		}
	}

	if (fpos >= len) {
		*start = buf;
		*eof = 1;
		return 0;
	}
	*start = buf + fpos;
	if ((len -= fpos) > length)
		return length;
	*eof = 1;
	return len;
}


void dump_jz_dma_channel(unsigned int dmanr)
{
	struct jz_dma_chan *chan;

	if (dmanr > MAX_DMA_NUM)
		return;
	chan = &jz_dma_table[dmanr];

	printk("DMA%d Registers:\n", dmanr);
	printk("  DMACR  = 0x%08x\n", REG_DMAC_DMACR(chan->io/HALF_DMA_NUM));
	printk("  DSAR   = 0x%08x\n", REG_DMAC_DSAR(dmanr));
	printk("  DTAR   = 0x%08x\n", REG_DMAC_DTAR(dmanr));
	printk("  DTCR   = 0x%08x\n", REG_DMAC_DTCR(dmanr));
	printk("  DRSR   = 0x%08x\n", REG_DMAC_DRSR(dmanr));
	printk("  DCCSR  = 0x%08x\n", REG_DMAC_DCCSR(dmanr));
	printk("  DCMD  = 0x%08x\n", REG_DMAC_DCMD(dmanr));
	printk("  DDA  = 0x%08x\n", REG_DMAC_DDA(dmanr));
	printk("  DMADBR = 0x%08x\n", REG_DMAC_DMADBR(chan->io/HALF_DMA_NUM));
}


/**
 * jz_request_dma - dynamically allcate an idle DMA channel to return
 * @dev_id: the specified dma device id or DMA_ID_RAW_SET
 * @dev_str: the specified dma device string name
 * @irqhandler: the irq handler, or NULL
 * @irqflags: the irq handler flags
 * @irq_dev_id: the irq handler device id for shared irq
 *
 * Finds a free channel, and binds the requested device to it.
 * Returns the allocated channel number, or negative on error.
 * Requests the DMA done IRQ if irqhandler != NULL.
 *
*/
/*int jz_request_dma(int dev_id, const char *dev_str,
		   void (*irqhandler)(int, void *, struct pt_regs *),
		   unsigned long irqflags,
		   void *irq_dev_id)
*/

int jz_request_dma(int dev_id, const char *dev_str,
		   irqreturn_t (*irqhandler)(int, void *),
		   unsigned long irqflags,
		   void *irq_dev_id)
{
	struct jz_dma_chan *chan;
	int i, ret;

	if (dev_id < 0 || dev_id >= DMA_ID_MAX)
		return -EINVAL;

	for (i = 0; i < MAX_DMA_NUM; i++) {
		    if (jz_dma_table[i].dev_id == dev_id)
			    break;
	}

	if (i == MAX_DMA_NUM) {
		for (i = 0; i < MAX_DMA_NUM; i++) {
			if (jz_dma_table[i].dev_id < 0)
				break;
		}
	}
	if (i == MAX_DMA_NUM)  /* no free channel */
		return -ENODEV;

	/* we got a free channel */
	chan = &jz_dma_table[i];

	if (irqhandler) {
		chan->irq = IRQ_DMA_0 + i;	// allocate irq number
		chan->irq_dev = irq_dev_id;
		if ((ret = request_irq(chan->irq, irqhandler, irqflags,
				       dev_str, chan->irq_dev))) {
			chan->irq = -1;
			chan->irq_dev = NULL;
			return ret;
		}
	} else {
		chan->irq = -1;
		chan->irq_dev = NULL;
	}

	// fill it in
	chan->io = i;
	chan->dev_id = dev_id;
	chan->dev_str = dev_str;
	chan->fifo_addr = dma_dev_table[dev_id].fifo_addr;
	chan->mode = dma_dev_table[dev_id].dma_mode;
	chan->source = dma_dev_table[dev_id].dma_source;

	if (i < HALF_DMA_NUM) {
		REG_DMAC_DMACKS(0) = 1 << i;
	} else {
		REG_DMAC_DMACKS(1) = 1 << (i - HALF_DMA_NUM);
	}

	return i;
}

/**
 * can be called while wait dma finish interrupt
 * can NOT be called from atomic or interrupt context
 *	(because we use schedule_timeout internally)
 **/
void jz_stop_dma(unsigned int chan)
{
	u32 old_counter = REG_DMAC_DTCR(chan);
	u32 cur_counter;

	/* wait for the counter not change */
	while (1) {
		schedule_timeout(HZ / 10); /* 100ms */
		cur_counter = REG_DMAC_DTCR(chan);
		if (cur_counter == old_counter)
			break;
		old_counter = cur_counter;
	}


	REG_DMAC_DCCSR(chan) = 0;

	REG_DMAC_DCMD(chan) = 0;
	REG_DMAC_DSAR(chan) = 0;
	REG_DMAC_DTAR(chan) = 0;
	REG_DMAC_DTCR(chan) = 0;
	REG_DMAC_DRSR(chan) = 0;
	REG_DMAC_DDA(chan) = 0;
}

void jz_free_dma(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan) {
		printk("Trying to free DMA%d\n", dmanr);
		return;
	}

	disable_dma(dmanr);
	if (chan->irq)
		free_irq(chan->irq, chan->irq_dev);

	chan->irq = -1;
	chan->irq_dev = NULL;
	chan->dev_id = -1;
}

void jz_set_dma_dest_width(int dmanr, int nbit)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	chan->mode &= ~DMAC_DCMD_DWDH_MASK;
	switch (nbit) {
	case 8:
		chan->mode |= DMAC_DCMD_DWDH_8;
		break;
	case 16:
		chan->mode |= DMAC_DCMD_DWDH_16;
		break;
	case 32:
		chan->mode |= DMAC_DCMD_DWDH_32;
		break;
	}
}

void jz_set_dma_src_width(int dmanr, int nbit)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	chan->mode &= ~DMAC_DCMD_SWDH_MASK;
	switch (nbit) {
	case 8:
		chan->mode |= DMAC_DCMD_SWDH_8;
		break;
	case 16:
		chan->mode |= DMAC_DCMD_SWDH_16;
		break;
	case 32:
		chan->mode |= DMAC_DCMD_SWDH_32;
		break;
	}
}

void jz_set_dma_block_size(int dmanr, int nbyte)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	chan->mode &= ~DMAC_DCMD_DS_MASK;
	switch (nbyte) {
	case 1:
		chan->mode |= DMAC_DCMD_DS_8BIT;
		break;
	case 2:
		chan->mode |= DMAC_DCMD_DS_16BIT;
		break;
	case 4:
		chan->mode |= DMAC_DCMD_DS_32BIT;
		break;
	case 16:
		chan->mode |= DMAC_DCMD_DS_16BYTE;
		break;
	case 32:
		chan->mode |= DMAC_DCMD_DS_32BYTE;
		break;
	}
}

unsigned int jz_get_dma_command(int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	return chan->mode;
}

/**
 * jz_set_dma_mode - do the raw settings for the specified DMA channel
 * @dmanr: the specified DMA channel
 * @mode: dma operate mode, DMA_MODE_READ or DMA_MODE_WRITE
 * @dma_mode: dma raw mode
 * @dma_source: dma raw request source
 * @fifo_addr: dma raw device fifo address
 *
 * Ensure call jz_request_dma(DMA_ID_RAW_SET, ...) first, then call
 * jz_set_dma_mode() rather than set_dma_mode() if you work with
 * and external request dma device.
 *
 * NOTE: Don not dynamically allocate dma channel if one external request
 *       dma device will occupy this channel.
*/
int jz_set_dma_mode(unsigned int dmanr, unsigned int mode,
		    unsigned int dma_mode, unsigned int dma_source,
		    unsigned int fifo_addr)
{
	int dev_id, i;
	struct jz_dma_chan *chan;

	if (dmanr > MAX_DMA_NUM)
		return -ENODEV;
	for (i = 0; i < MAX_DMA_NUM; i++) {
		if (jz_dma_table[i].dev_id < 0)
			break;
	}
	if (i == MAX_DMA_NUM)
		return -ENODEV;

	chan = &jz_dma_table[dmanr];
	dev_id = chan->dev_id;
	if (dev_id > 0) {
		printk(KERN_DEBUG "%s sets the allocated DMA channel %d!\n",
		       __FUNCTION__, dmanr);
		return -ENODEV;
	}

	/* clone it from the dynamically allocated. */
	if (i != dmanr) {
		chan->irq = jz_dma_table[i].irq;
		chan->irq_dev = jz_dma_table[i].irq_dev;
		chan->dev_str = jz_dma_table[i].dev_str;
		jz_dma_table[i].irq = 0;
		jz_dma_table[i].irq_dev = NULL;
		jz_dma_table[i].dev_id = -1;
	}
	chan->dev_id = DMA_ID_RAW_SET;
	chan->io = dmanr;
	chan->fifo_addr = fifo_addr;
	chan->mode = dma_mode;
	chan->source = dma_source;

	set_dma_mode(dmanr, dma_mode);

	return dmanr;
}

void enable_dma(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	REG_DMAC_DCCSR(dmanr) &= ~(DMAC_DCCSR_HLT | DMAC_DCCSR_TT | DMAC_DCCSR_AR);
	REG_DMAC_DCCSR(dmanr) |= DMAC_DCCSR_NDES; /* No-descriptor transfer */
	__dmac_enable_channel(dmanr);
	if (chan->irq)
		__dmac_channel_enable_irq(dmanr);
}

#define DMA_DISABLE_POLL 0x10000

void disable_dma(unsigned int dmanr)
{
	int i;
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	if (!__dmac_channel_enabled(dmanr))
		return;

	for (i = 0; i < DMA_DISABLE_POLL; i++)
		if (__dmac_channel_transmit_end_detected(dmanr))
			break;
#if 0
	if (i == DMA_DISABLE_POLL)
		printk(KERN_INFO "disable_dma: poll expired!\n");
#endif

	__dmac_disable_channel(dmanr);
	if (chan->irq)
		__dmac_channel_disable_irq(dmanr);
}

/* Note: DMA_MODE_MASK is simulated by sw */
void set_dma_mode(unsigned int dmanr, unsigned int mode)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	chan->mode |= mode & ~(DMAC_DCMD_SAI | DMAC_DCMD_DAI);
	mode &= DMA_MODE_MASK;
	if (mode == DMA_MODE_READ) {
		chan->mode |= DMAC_DCMD_DAI;
		chan->mode &= ~DMAC_DCMD_SAI;
	} else if (mode == DMA_MODE_WRITE) {
		chan->mode |= DMAC_DCMD_SAI;
		chan->mode &= ~DMAC_DCMD_DAI;
	} else {
		printk(KERN_DEBUG "set_dma_mode() just supports DMA_MODE_READ or DMA_MODE_WRITE!\n");
	}
	REG_DMAC_DCMD(chan->io) = chan->mode & ~DMA_MODE_MASK;
	REG_DMAC_DRSR(chan->io) = chan->source;
}

void set_dma_addr(unsigned int dmanr, unsigned int phyaddr)
{
	unsigned int mode;
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	mode = chan->mode & DMA_MODE_MASK;
	if (mode == DMA_MODE_READ) {
		REG_DMAC_DSAR(chan->io) = chan->fifo_addr;
		REG_DMAC_DTAR(chan->io) = phyaddr;
	} else if (mode == DMA_MODE_WRITE) {
		REG_DMAC_DSAR(chan->io) = phyaddr;
		REG_DMAC_DTAR(chan->io) = chan->fifo_addr;
	} else
		printk(KERN_DEBUG "Driver should call set_dma_mode() ahead set_dma_addr()!\n");
}

void set_dma_count(unsigned int dmanr, unsigned int bytecnt)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	int dma_ds[] = {4, 1, 2, 16, 32};
	unsigned int ds;

	if (!chan)
		return;

	ds = (chan->mode & DMAC_DCMD_DS_MASK) >> DMAC_DCMD_DS_BIT;
	REG_DMAC_DTCR(chan->io) = bytecnt / dma_ds[ds]; // transfer count
}

unsigned int get_dma_residue(unsigned int dmanr)
{
	unsigned int count, ds;
	int dma_ds[] = {4, 1, 2, 16, 32};
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return 0;

	ds = (chan->mode & DMAC_DCMD_DS_MASK) >> DMAC_DCMD_DS_BIT;
	count = REG_DMAC_DTCR(chan->io);
	count = count * dma_ds[ds];

	return count;
}

void jz_set_oss_dma(unsigned int dmanr, unsigned int mode, unsigned int audio_fmt)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	switch (audio_fmt) {
	case AFMT_U8:
		/* burst mode : 32BIT */
		break;
	case AFMT_S16_LE:
		/* burst mode : 16BYTE */
		if (mode == DMA_MODE_READ) {
			chan->mode = DMA_AIC_32_16BYTE_RX_CMD | DMA_MODE_READ;
			chan->mode |= mode & ~(DMAC_DCMD_SAI | DMAC_DCMD_DAI);
			mode &= DMA_MODE_MASK;
			chan->mode |= DMAC_DCMD_DAI;
			chan->mode &= ~DMAC_DCMD_SAI;
		} else if (mode == DMA_MODE_WRITE) {
			chan->mode = DMA_AIC_32_16BYTE_TX_CMD | DMA_MODE_WRITE;
			//chan->mode = DMA_AIC_16BYTE_TX_CMD | DMA_MODE_WRITE;
			chan->mode |= mode & ~(DMAC_DCMD_SAI | DMAC_DCMD_DAI);
			mode &= DMA_MODE_MASK;
			chan->mode |= DMAC_DCMD_SAI;
			chan->mode &= ~DMAC_DCMD_DAI;
		} else
			printk("oss_dma_burst_mode() just supports DMA_MODE_READ or DMA_MODE_WRITE!\n");

		REG_DMAC_DCMD(chan->io) = chan->mode & ~DMA_MODE_MASK;
		REG_DMAC_DRSR(chan->io) = chan->source;
		break;
	}
}

void jz_set_alsa_dma(unsigned int dmanr, unsigned int mode, unsigned int audio_fmt)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	switch (audio_fmt) {
	case 8:
		/* SNDRV_PCM_FORMAT_S8 burst mode : 32BIT */
		break;
	case 16:
		/* SNDRV_PCM_FORMAT_S16_LE burst mode : 16BYTE */
		if (mode == DMA_MODE_READ) {
			chan->mode = DMA_AIC_16BYTE_RX_CMD | DMA_MODE_READ;
			chan->mode |= mode & ~(DMAC_DCMD_SAI | DMAC_DCMD_DAI);
			mode &= DMA_MODE_MASK;
			chan->mode |= DMAC_DCMD_DAI;
			chan->mode &= ~DMAC_DCMD_SAI;
		} else if (mode == DMA_MODE_WRITE) {
			chan->mode = DMA_AIC_16BYTE_TX_CMD | DMA_MODE_WRITE;
			chan->mode |= mode & ~(DMAC_DCMD_SAI | DMAC_DCMD_DAI);
			mode &= DMA_MODE_MASK;
			chan->mode |= DMAC_DCMD_SAI;
			chan->mode &= ~DMAC_DCMD_DAI;
		} else
			printk("alsa_dma_burst_mode() just supports DMA_MODE_READ or DMA_MODE_WRITE!\n");

		REG_DMAC_DCMD(chan->io) = chan->mode & ~DMA_MODE_MASK;
		REG_DMAC_DRSR(chan->io) = chan->source;
		break;
	}
}

//#define JZ4770_DMAC_TEST_ENABLE
#undef JZ4770_DMAC_TEST_ENABLE

#ifdef JZ4770_DMAC_TEST_ENABLE

/*
 * DMA test: external address <--> external address
 */
#define TEST_DMA_SIZE  16*1024

static jz_dma_desc *dma_desc;

static int dma_chan;
static dma_addr_t dma_desc_phys_addr;
static unsigned int dma_src_addr, dma_src_phys_addr, dma_dst_addr, dma_dst_phys_addr;

static int dma_check_result(void *src, void *dst, int size)
{
	unsigned int addr1, addr2, i, err = 0;

	addr1 = (unsigned int)src;
	addr2 = (unsigned int)dst;

	for (i = 0; i < size; i += 4) {
		if (*(volatile unsigned int *)addr1 != *(volatile unsigned int *)addr2) {
			err++;
			printk("wrong data at 0x%08x: src 0x%08x  dst 0x%08x\n", addr2, *(volatile unsigned int *)addr1, *(volatile unsigned int *)addr2);
		}
		addr1 += 4;
		addr2 += 4;
	}
	printk("check DMA result err=%d\n", err);
	return err;
}

static irqreturn_t jz4770_dma_irq(int irq, void *dev_id)
{
	printk("jz4770_dma_irq %d\n", irq);


	if (__dmac_channel_transmit_halt_detected(dma_chan)) {
		printk("DMA HALT\n");
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
		__dmac_channel_clear_transmit_halt(dma_chan);
	}

	if (__dmac_channel_address_error_detected(dma_chan)) {
		printk("DMA ADDR ERROR\n");
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
		REG_DMAC_DSAR(dma_chan) = 0; /* clear source address register */
		REG_DMAC_DTAR(dma_chan) = 0; /* clear target address register */
		__dmac_channel_clear_address_error(dma_chan);
	}

	if (__dmac_channel_descriptor_invalid_detected(dma_chan)) {
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
		printk("DMA DESC INVALID\n");
		__dmac_channel_clear_descriptor_invalid(dma_chan);
	}

	if (__dmac_channel_count_terminated_detected(dma_chan)) {
		printk("DMA CT\n");
		__dmac_channel_clear_count_terminated(dma_chan);
	}

	if (__dmac_channel_transmit_end_detected(dma_chan)) {
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
		printk("DMA TT\n");
		__dmac_channel_clear_transmit_end(dma_chan);
		dump_jz_dma_channel(dma_chan);
		dma_check_result((void *)dma_src_addr, (void *)dma_dst_addr, TEST_DMA_SIZE);
	}

	return IRQ_HANDLED;
}

void dma_nodesc_test(void)
{
	unsigned int addr, i;

	printk("dma_nodesc_test\n");

	/* Request DMA channel and setup irq handler */
	dma_chan = jz_request_dma(DMA_ID_AUTO, "auto", jz4770_dma_irq,
				  IRQF_DISABLED, NULL);
	if (dma_chan < 0) {
		printk("Setup irq failed\n");
		return;
	}

	printk("Requested DMA channel = %d\n", dma_chan);

	/* Allocate DMA buffers */
	dma_src_addr = __get_free_pages(GFP_KERNEL, 2); /* 16KB */
	dma_dst_addr = __get_free_pages(GFP_KERNEL, 2); /* 16KB */

	dma_src_phys_addr = CPHYSADDR(dma_src_addr);
	dma_dst_phys_addr = CPHYSADDR(dma_dst_addr);

	printk("Buffer addresses: 0x%08x 0x%08x 0x%08x 0x%08x\n",
	       dma_src_addr, dma_src_phys_addr, dma_dst_addr, dma_dst_phys_addr);

	/* Prepare data for source buffer */
	addr = (unsigned int)dma_src_addr;
	for (i = 0; i < TEST_DMA_SIZE; i += 4) {
		*(volatile unsigned int *)addr = addr;
		addr += 4;
	}
	dma_cache_wback((unsigned long)dma_src_addr, TEST_DMA_SIZE);

	/* Init target buffer */
	memset((void *)dma_dst_addr, 0, TEST_DMA_SIZE);
	dma_cache_wback((unsigned long)dma_dst_addr, TEST_DMA_SIZE);

	/* Init DMA module */
	printk("Starting DMA\n");
	REG_DMAC_DMACR(dma_chan/HALF_DMA_NUM) = 0;
	REG_DMAC_DCCSR(dma_chan) = 0;
	REG_DMAC_DRSR(dma_chan) = DMAC_DRSR_RS_AUTO;
	REG_DMAC_DSAR(dma_chan) = dma_src_phys_addr;
	REG_DMAC_DTAR(dma_chan) = dma_dst_phys_addr;
	REG_DMAC_DTCR(dma_chan) = 512;
	REG_DMAC_DCMD(dma_chan) = DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BYTE | DMAC_DCMD_TIE;
	REG_DMAC_DCCSR(dma_chan) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
	REG_DMAC_DMACR(dma_chan/HALF_DMA_NUM) = DMAC_DMACR_DMAE; /* global DMA enable bit */

	/* wait a long time, ensure transfer end */
	printk("wait 3s...\n");
	mdelay(3000);		/* wait 3s */

	REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
	/* free buffers */
	printk("free DMA buffers\n");
	free_pages(dma_src_addr, 2);
	free_pages(dma_dst_addr, 2);

	if (dma_desc)
		free_pages((unsigned int)dma_desc, 0);

	/* free dma */
	jz_free_dma(dma_chan);
}

void dma_desc_test(void)
{
	unsigned int next, addr, i;
	static jz_dma_desc *desc;

	printk("dma_desc_test\n");

	/* Request DMA channel and setup irq handler */
	dma_chan = jz_request_dma(DMA_ID_AUTO, "auto", jz4770_dma_irq,
				  IRQF_DISABLED, NULL);
	if (dma_chan < 0) {
		printk("Setup irq failed\n");
		return;
	}

	printk("Requested DMA channel = %d\n", dma_chan);

	/* Allocate DMA buffers */
	dma_src_addr = __get_free_pages(GFP_KERNEL, 2); /* 16KB */
	dma_dst_addr = __get_free_pages(GFP_KERNEL, 2); /* 16KB */

	dma_src_phys_addr = CPHYSADDR(dma_src_addr);
	dma_dst_phys_addr = CPHYSADDR(dma_dst_addr);

	printk("Buffer addresses: 0x%08x 0x%08x 0x%08x 0x%08x\n",
	       dma_src_addr, dma_src_phys_addr, dma_dst_addr, dma_dst_phys_addr);

	/* Prepare data for source buffer */
	addr = (unsigned int)dma_src_addr;
	for (i = 0; i < TEST_DMA_SIZE; i += 4) {
		*(volatile unsigned int *)addr = addr;
		addr += 4;
	}
	dma_cache_wback((unsigned long)dma_src_addr, TEST_DMA_SIZE);

	/* Init target buffer */
	memset((void *)dma_dst_addr, 0, TEST_DMA_SIZE);
	dma_cache_wback((unsigned long)dma_dst_addr, TEST_DMA_SIZE);

	/* Allocate DMA descriptors */
	dma_desc = (jz_dma_desc *)__get_free_pages(GFP_KERNEL, 0);
	dma_desc_phys_addr = CPHYSADDR((unsigned long)dma_desc);

	printk("DMA descriptor address: 0x%08x  0x%08x\n", (u32)dma_desc, dma_desc_phys_addr);

	/* Setup DMA descriptors */
	desc = dma_desc;
	next = (dma_desc_phys_addr + (sizeof(jz_dma_desc))) >> 4;

	desc->dcmd = DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BYTE | DMAC_DCMD_DES_V | DMAC_DCMD_DES_VM | DMAC_DCMD_DES_VIE | DMAC_DCMD_TIE | DMAC_DCMD_LINK;
	desc->dsadr = dma_src_phys_addr;    /* DMA source address */
	desc->dtadr = dma_dst_phys_addr;    /* DMA target address */
	desc->ddadr = (next << 24) + 128;    /* size: 128*32 bytes = 4096 bytes */

	desc++;
	next = (dma_desc_phys_addr + 2*(sizeof(jz_dma_desc))) >> 4;

	desc->dcmd = DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_16BYTE | DMAC_DCMD_DES_V | DMAC_DCMD_DES_VM | DMAC_DCMD_DES_VIE | DMAC_DCMD_TIE | DMAC_DCMD_LINK;
	desc->dsadr = dma_src_phys_addr + 4096;	/* DMA source address */
	desc->dtadr = dma_dst_phys_addr + 4096;	/* DMA target address */
	desc->ddadr = (next << 24) + 256;    /* size: 256*16 bytes = 4096 bytes */

	desc++;
	next = (dma_desc_phys_addr + 3*(sizeof(jz_dma_desc))) >> 4;

	desc->dcmd = DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_16BYTE | DMAC_DCMD_DES_V | DMAC_DCMD_DES_VM | DMAC_DCMD_DES_VIE | DMAC_DCMD_TIE | DMAC_DCMD_LINK;
	desc->dsadr = dma_src_phys_addr + 8192;	/* DMA source address */
	desc->dtadr = dma_dst_phys_addr + 8192;	/* DMA target address */
	desc->ddadr = (next << 24) + 256;    /* size: 256*16 bytes = 4096 bytes */

	desc++;
	next = (dma_desc_phys_addr + 4*(sizeof(jz_dma_desc))) >> 4;

	desc->dcmd = DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_DES_V | DMAC_DCMD_DES_VM | DMAC_DCMD_DES_VIE | DMAC_DCMD_TIE;
	desc->dsadr = dma_src_phys_addr + 12*1024;	/* DMA source address */
	desc->dtadr = dma_dst_phys_addr + 12*1024;	/* DMA target address */
	desc->ddadr = (next << 24) + 1024;    /* size: 1024*4 bytes = 4096 bytes */

	dma_cache_wback((unsigned long)dma_desc, 4*(sizeof(jz_dma_desc)));

	/* Setup DMA descriptor address */
	REG_DMAC_DDA(dma_chan) = dma_desc_phys_addr;

	/* Setup request source */
	REG_DMAC_DRSR(dma_chan) = DMAC_DRSR_RS_AUTO;

	/* Setup DMA channel control/status register */
	REG_DMAC_DCCSR(dma_chan) = DMAC_DCCSR_EN;	/* descriptor transfer, clear status, start channel */

	/* Enable DMA */
	REG_DMAC_DMACR(dma_chan/HALF_DMA_NUM) = DMAC_DMACR_DMAE;

	/* DMA doorbell set -- start DMA now ... */
	REG_DMAC_DMADBSR(dma_chan/HALF_DMA_NUM) = 1 << dma_chan;

	/* wait a long time, ensure transfer end */
	printk("wait 3s...\n");
	mdelay(3000);		/* wait 3s */

	REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
	/* free buffers */
	printk("free DMA buffers\n");
	free_pages(dma_src_addr, 2);
	free_pages(dma_dst_addr, 2);

	if (dma_desc)
		free_pages((unsigned int)dma_desc, 0);

	/* free dma */
	jz_free_dma(dma_chan);
}

/*
 * channel 0: read
 * channel 1: write
 * read and write are simutanously
 */
void dma_two_desc_test(void) {

}

#endif

//EXPORT_SYMBOL_NOVERS(jz_dma_table);
EXPORT_SYMBOL(jz_dma_table);
EXPORT_SYMBOL(jz_request_dma);
EXPORT_SYMBOL(jz_free_dma);
EXPORT_SYMBOL(jz_set_dma_src_width);
EXPORT_SYMBOL(jz_set_dma_dest_width);
EXPORT_SYMBOL(jz_set_dma_block_size);
EXPORT_SYMBOL(jz_set_dma_mode);
EXPORT_SYMBOL(set_dma_mode);
EXPORT_SYMBOL(jz_set_oss_dma);
EXPORT_SYMBOL(jz_set_alsa_dma);
EXPORT_SYMBOL(set_dma_addr);
EXPORT_SYMBOL(set_dma_count);
EXPORT_SYMBOL(get_dma_residue);
EXPORT_SYMBOL(enable_dma);
EXPORT_SYMBOL(disable_dma);
EXPORT_SYMBOL(dump_jz_dma_channel);

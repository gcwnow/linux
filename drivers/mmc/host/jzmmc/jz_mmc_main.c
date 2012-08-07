/*
 *  linux/drivers/mmc/host/jz_mmc/jz_mmc_main.c - JZ SD/MMC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/card.h>
#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/pm.h>
#include <linux/scatterlist.h>
#include <asm/io.h>
#include <asm/scatterlist.h>

#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770msc.h>

#include "include/jz_mmc_host.h"
#include "include/jz_mmc_controller.h"


struct jz_mmc_controller controller[JZ_MAX_MSC_NUM];

void jz_mmc_finish_request(struct jz_mmc_host *host, struct mmc_request *mrq)
{
	host->curr_mrq = NULL;
	up(&host->mutex);
	mmc_request_done(host->mmc, mrq);
}

static void jz_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	struct jz_mmc_functions *functions = host->plat->driver_data;

	down(&host->mutex);

	if (SD_IO_SEND_OP_COND == mrq->cmd->opcode) {
		if(host->plat->support_sdio == 0) {
			mrq->cmd->error = -ETIMEDOUT;
			jz_mmc_finish_request(host, mrq);
			return;
		}
	}

	if (host->eject) {
		if (mrq->data && !(mrq->data->flags & MMC_DATA_READ)) {
			mrq->cmd->error = -EIO;
			mrq->data->bytes_xfered = 0;
		} else
			mrq->cmd->error = -ENOMEDIUM;
		up(&host->mutex);
		mmc_request_done(mmc, mrq);
		return;
	}

	BUG_ON (host->curr_mrq);
	host->curr_mrq = mrq;
	functions->execute_cmd(host);
	jz_mmc_finish_request(host, mrq);
}

static int jz_mmc_get_ro(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	if(host->plat->write_protect != NULL)
		return host->plat->write_protect(mmc_dev(host->mmc));
	else
		return 0;
}

static int jz_mmc_get_cd(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	if(host->plat->status != NULL) {
		return host->plat->status(mmc_dev(host->mmc));
	}
	else
		return 1;
}

/* set clock and power */
static void jz_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	struct jz_mmc_functions *functions = host->plat->driver_data;
	//void *dev;

	if(!functions) {
		printk(KERN_ERR "%s: functions is NULL!\n", __FUNCTION__);
		return;
	}

	if (ios->clock) {
		functions->set_clock(host, ios->clock);
	}

	switch(ios->power_mode) {
	case MMC_POWER_ON:
		host->plat->power_on(NULL);
		host->cmdat |= MSC_CMDAT_INIT;
		break;
	case MMC_POWER_OFF:
		host->plat->power_off(NULL);
		break;
	default:
		break;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4) {

		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;

		if(host->plat->bus_width == 4)
			host->cmdat |= MSC_CMDAT_BUS_WIDTH_4BIT;
		else
			host->cmdat |= host->plat->bus_width;
	} else if (ios->bus_width == MMC_BUS_WIDTH_8) {

		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;

		if(host->plat->bus_width == 8)
			host->cmdat |= MSC_CMDAT_BUS_WIDTH_8BIT;
//		else
//			host->cmdat |= host->plat->bus_width;
	} else {
		/* 1 bit bus*/
		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_8BIT;
	}
}

static const struct mmc_host_ops jz_mmc_ops = {
	.request = jz_mmc_request,
	.get_ro = jz_mmc_get_ro,
	.set_ios = jz_mmc_set_ios,
	.get_cd = jz_mmc_get_cd,
};

#ifdef MSC_DEBUG_DMA
static struct jz_mmc_host *msc_hosts[JZ_MAX_MSC_NUM] = { NULL, NULL, NULL };

static void dump_host_info(struct jz_mmc_host *host) {
	int i = 0;
	JZ_MSC_DMA_DESC *desc = NULL;

	printk("*** msc%d host info ***\n", host->pdev_id);
	dump_jz_dma_channel(host->dma.channel);
	printk("*** last running descriptors = %d direction = %d ***\n", host->num_desc, host->last_direction);
	desc = host->dma_desc;
	for (i = 0; i < host->num_desc; i++) {
		printk("desc address = %p\n", desc + i);
		printk("dcmd = 0x%08x\n", desc[i].dcmd);
		printk("dsadr = 0x%08x\n", desc[i].dsadr);
		printk("dtadr = 0x%08x\n", desc[i].dtadr);
		printk("ddadr = 0x%08x\n", desc[i].ddadr);
		printk("dstrd = 0x%08x\n", desc[i].dstrd);
		printk("dreqt = 0x%08x\n", desc[i].dreqt);
		printk("resv0 = 0x%08x\n", desc[i].reserved0);
		printk("resv1 = 0x%08x\n", desc[i].reserved1);
		printk("==========\n");
	}

	printk("curr tx_ack = %d\n", host->tx_ack);
	printk("curr rx_ack = %d\n", host->rx_ack);
}

void msc_dump_host_info(void) {
	int i = 0;

	for (i = 0; i < JZ_MAX_MSC_NUM; i++) {
		if (msc_hosts[i] != NULL) {
			dump_host_info(msc_hosts[0]);
		}
	}
}
EXPORT_SYMBOL(msc_dump_host_info);
#endif	/* MSC_DEBUG_DMA */

static int jz_mmc_probe(struct platform_device *pdev)
{
	struct jz_mmc_platform_data *plat = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct jz_mmc_host *host = NULL;
	struct jz_mmc_functions *functions;

	struct resource *irqres = NULL;
	struct resource *memres = NULL;
	struct resource *dmares = NULL;
	int i;

	char clk_name[5];

	if (pdev == NULL) {
		printk(KERN_ERR "%s: pdev is NULL\n", __func__);
		return -EINVAL;
	}
	if (!plat) {
		printk(KERN_ERR "%s: Platform data not available\n", __func__);
		return -EINVAL;
	}

	if (JZ_MSC_ID_INVALID(pdev->id))
		return -EINVAL;

	if (pdev->resource == NULL || pdev->num_resources < 2) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}
	for (i = 0; i < pdev->num_resources; i++) {
		if (pdev->resource[i].flags & IORESOURCE_MEM)
			memres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_IRQ)
			irqres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_DMA)
			dmares = &pdev->resource[i];
	}
	if (!irqres || !memres) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}
	/*
	 * Setup our host structure
	 */
	mmc = mmc_alloc_host(sizeof(struct jz_mmc_host), &pdev->dev);
	if (!mmc) {
		return -ENOMEM;
	}
	host = mmc_priv(mmc);
	host->pdev_id = pdev->id;
	host->plat = plat;
	host->mmc = mmc;

	sprintf(clk_name, "mmc%i", pdev->id);
	host->clk = clk_get(&pdev->dev, clk_name);
	if (IS_ERR(host->clk))
		return -ENXIO;

	clk_enable(host->clk);

#if 0			      /* Lutts */
	// base address of MSC controller
	host->base = ioremap(memres->start, PAGE_SIZE);
	if (!host->base) {
		return -ENOMEM;
	}
#endif
	host->irq = irqres->start;
	if (dmares)
		host->dma_id = dmares->start;
	else
		host->dma_id = -1;
	//spin_lock_init(&host->lock);
	sema_init(&host->mutex, 1);

	/*
	 * Setup MMC host structure
	 */
	mmc->ops = &jz_mmc_ops;
	mmc->f_min = MMC_CLOCK_SLOW;
	mmc->f_max = SD_CLOCK_HIGH;
	mmc->ocr_avail = plat->ocr_mask;
	mmc->caps |= host->plat->max_bus_width;
	mmc->max_segs = NR_SG;
	mmc->max_blk_size = 4095;
	mmc->max_blk_count = 65535;

	mmc->max_req_size = PAGE_SIZE * 16;
	mmc->max_seg_size = mmc->max_req_size;
	plat->init(&pdev->dev);
	plat->power_on(&pdev->dev);
	/*
	 * Initialize controller and register some functions
	 * From here, we can do everything!
	 */
	controller_register(&controller[host->pdev_id], host);
	functions = host->plat->driver_data;
	if(controller[host->pdev_id].init(&controller[host->pdev_id], host, pdev))
		goto out;
	mmc_set_drvdata(pdev, mmc);
	mmc_add_host(mmc);
#ifdef MSC_DEBUG_DMA
	msc_hosts[host->pdev_id] = host;
#endif

	printk("JZ %s driver registered\n", mmc_hostname(host->mmc));

	return 0;

out:
	return -1;
}

static int jz_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct jz_mmc_platform_data *plat = pdev->dev.platform_data;

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		struct jz_mmc_host *host = mmc_priv(mmc);
		struct jz_mmc_functions *functions = host->plat->driver_data;

		plat->power_off(&pdev->dev);

		functions->deinit(host, pdev);

		clk_put(host->clk);

		mmc_remove_host(mmc);
		mmc_free_host(mmc);
	}
	return 0;
}

#ifdef CONFIG_PM
static int jz_mmc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct jz_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

	host->sleeping = 1;

	if (mmc) {
		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO) {
			ret = mmc_suspend_host(mmc);
		}

		clk_disable(host->clk);
	}
	return ret;
}

extern int jz_mmc_detect(struct jz_mmc_host *host, int from_resuming);
static int jz_mmc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct jz_mmc_host *host = mmc_priv(mmc);

#ifdef CONFIG_JZ_SYSTEM_AT_CARD
	if (host->pdev_id == 0){
		mmc_resume_host(host->mmc);
		jz_mmc_reset(host);
		if(cpm_get_clock(CGU_MSC0CLK) > SD_CLOCK_FAST)
			REG_MSC_LPM(host->pdev_id) |= 1<<31;
		return 0;
	}
#endif

	if (mmc) {
		clk_enable(host->clk);

		if ( (mmc->card == NULL) || (mmc->card->type != MMC_TYPE_SDIO) )
			jz_mmc_detect(host, 1);
	}

	return 0;
}
#else
#define jz_mmc_suspend      NULL
#define jz_mmc_resume       NULL
#endif

static struct platform_driver jz_msc_driver = {
	.probe = jz_mmc_probe,
	.remove = jz_mmc_remove,
	.suspend = jz_mmc_suspend,
	.resume = jz_mmc_resume,
	.driver = {
		   .name = "jz-msc",
		   },
};

module_platform_driver(jz_msc_driver);

MODULE_DESCRIPTION("JZ47XX SD/Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");

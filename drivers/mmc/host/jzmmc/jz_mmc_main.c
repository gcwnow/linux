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
#include <linux/gpio.h>
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
#include <asm/mach-jz4770/mmc.h>

#include "include/chip-msc.h"
#include "include/jz_mmc_dma.h"
#include "include/jz_mmc_gpio.h"
#include "include/jz_mmc_host.h"
#include "include/jz_mmc_msc.h"


#define JZ_MAX_MSC_NUM 3

void jz_mmc_finish_request(struct jz_mmc_host *host, struct mmc_request *mrq)
{
	host->curr_mrq = NULL;
	up(&host->mutex);
	mmc_request_done(host->mmc, mrq);
}

static void jz_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	down(&host->mutex);

	if (SD_IO_SEND_OP_COND == mrq->cmd->opcode) {
		if(host->pdata->support_sdio == 0) {
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
	jz_mmc_execute_cmd(host);
	jz_mmc_finish_request(host, mrq);
}

static int jz_mmc_get_ro(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	if (!gpio_is_valid(host->pdata->gpio_read_only))
		return -ENOSYS;

	return gpio_get_value(host->pdata->gpio_read_only) ^
		host->pdata->read_only_active_low;
}

static int jz_mmc_get_cd(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	if (!gpio_is_valid(host->pdata->gpio_card_detect))
		return -ENOSYS;

	return gpio_get_value(host->pdata->gpio_card_detect) ^
			host->pdata->card_detect_active_low;
}

/* set clock and power */
static void jz_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	//void *dev;

	if (ios->clock)
		jz_mmc_set_clock(host, ios->clock);

	switch(ios->power_mode) {
	case MMC_POWER_ON:
		if (gpio_is_valid(host->pdata->gpio_power))
			gpio_set_value(host->pdata->gpio_power,
				       !host->pdata->power_active_low);
		host->cmdat |= MSC_CMDAT_INIT;
		break;
	case MMC_POWER_OFF:
		if (gpio_is_valid(host->pdata->gpio_power))
			gpio_set_value(host->pdata->gpio_power,
				       host->pdata->power_active_low);
		break;
	default:
		break;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4) {

		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;

		if(host->pdata->bus_width == 4)
			host->cmdat |= MSC_CMDAT_BUS_WIDTH_4BIT;
		else
			host->cmdat |= host->pdata->bus_width;
	} else if (ios->bus_width == MMC_BUS_WIDTH_8) {

		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;

		if(host->pdata->bus_width == 8)
			host->cmdat |= MSC_CMDAT_BUS_WIDTH_8BIT;
//		else
//			host->cmdat |= host->pdata->bus_width;
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

static int jz_mmc_probe(struct platform_device *pdev)
{
	int ret;
	struct jz_mmc_platform_data *pdata = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct jz_mmc_host *host;

	struct resource *dmares;

	char clk_name[5];

	if (pdev->id < 0 || pdev->id >= JZ_MAX_MSC_NUM)
		return -EINVAL;

	mmc = mmc_alloc_host(sizeof(struct jz_mmc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "Failed to alloc mmc host structure\n");
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->pdata = pdata;

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = host->irq;
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
		goto err_free_host;
	}

	sprintf(clk_name, "mmc%i", pdev->id);
	host->clk = devm_clk_get(&pdev->dev, clk_name);
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		dev_err(&pdev->dev, "Failed to get mmc clock\n");
		goto err_free_host;
	}

	// TODO: larsc's driver does this in the set_ios handler.
	clk_enable(host->clk);

	host->base = devm_ioremap_resource(&pdev->dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		dev_err(&pdev->dev, "Failed to get and remap mmio region\n");
		goto err_free_host;
	}

	dmares = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!dmares) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform dma\n");
		goto err_free_host;
	}
	host->dma_id = dmares->start;

	/*
	 * Setup MMC host structure
	 */
	mmc->ops = &jz_mmc_ops;
	mmc->f_min = MMC_CLOCK_SLOW;
	mmc->f_max = SD_CLOCK_HIGH;
	mmc->ocr_avail = pdata->ocr_mask;
	mmc->caps |= host->pdata->max_bus_width;

	mmc->max_blk_size = 4095;
	mmc->max_blk_count = 65535;
	mmc->max_req_size = PAGE_SIZE * 16;

	mmc->max_segs = 1;
	mmc->max_seg_size = mmc->max_req_size;

	host->pdev_id = pdev->id;
	host->mmc = mmc;
	//spin_lock_init(&host->lock);
	sema_init(&host->mutex, 1);

	ret = jz_mmc_msc_init(host);
	if (ret)
		goto err_free_host;

	ret = jz_mmc_gpio_init(host, pdev);
	if (ret)
		goto err_deinit_msc;

	ret = jz_mmc_init_dma(host);
	if (ret)
		goto err_deinit_gpio;

	if (gpio_is_valid(host->pdata->gpio_power))
		gpio_set_value(host->pdata->gpio_power,
			       !host->pdata->power_active_low);

	platform_set_drvdata(pdev, mmc);
	ret = mmc_add_host(mmc);

	if (ret) {
		dev_err(&pdev->dev, "Failed to add mmc host: %d\n", ret);
		goto err_deinit_dma;
	}
	dev_info(&pdev->dev, "JZ SD/MMC card driver registered\n");

	return 0;

err_deinit_dma:
	jz_mmc_deinit_dma(host);
err_deinit_gpio:
	jz_mmc_gpio_deinit(host, pdev);
err_deinit_msc:
	jz_mmc_msc_deinit(host);
err_free_host:
	mmc_free_host(mmc);

	return ret;
}

static int jz_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	if (mmc) {
		struct jz_mmc_host *host = mmc_priv(mmc);

		if (gpio_is_valid(host->pdata->gpio_power))
			gpio_set_value(host->pdata->gpio_power,
				       host->pdata->power_active_low);

		jz_mmc_deinit_dma(host);
		jz_mmc_gpio_deinit(host, pdev);
		jz_mmc_msc_deinit(host);

		mmc_remove_host(mmc);
		mmc_free_host(mmc);
	}
	return 0;
}

#ifdef CONFIG_PM

static int jz_mmc_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
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

static int jz_mmc_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
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
			if (host->card_detect_irq >= 0)
				jz_mmc_detect(host, 1);
	}

	return 0;
}

const struct dev_pm_ops jz_mmc_pm_ops = {
	.suspend	= jz_mmc_suspend,
	.resume		= jz_mmc_resume,
	.poweroff	= jz_mmc_suspend,
	.restore	= jz_mmc_resume,
};

#define JZ_MMC_PM_OPS (&jz_mmc_pm_ops)
#else
#define JZ_MMC_PM_OPS NULL
#endif

static struct platform_driver jz_msc_driver = {
	.probe		= jz_mmc_probe,
	.remove		= jz_mmc_remove,
	.driver = {
		.name	= "jz-msc",
		.owner	= THIS_MODULE,
		.pm	= JZ_MMC_PM_OPS,
	},
};

module_platform_driver(jz_msc_driver);

MODULE_DESCRIPTION("JZ47XX SD/Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");

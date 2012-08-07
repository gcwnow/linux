/*
 *  linux/drivers/mmc/host/jz_mmc/controller/jz_mmc_controller.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/dma-mapping.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/scatterlist.h>

#include <asm/mach-jz4770/jz4770msc.h>

#include "include/jz_mmc_msc.h"
#include "include/jz_mmc_controller.h"


static int data_transmit_dma(struct jz_mmc_host *host)
{
	jz_mmc_data_start(host);

	return 0;
}

static int controller_init(struct jz_mmc_controller *controller, struct jz_mmc_host *host,
				struct platform_device *pdev)
{
	int ret = 0;

	ret = controller->msc.init(host);
	if(ret) {
		return ret;
	}

	ret = controller->gpio.init(host, pdev);
	if(ret) {
		goto gpio_failed;
	}

	ret = controller->dma.init(host);
	if(ret) {
		goto dma_failed;
	}

	return 0;

dma_failed:
	controller->gpio.deinit(host, pdev);
gpio_failed:
	controller->msc.deinit(host);
	return ret;
}

static void controller_deinit(struct jz_mmc_host *host, struct platform_device *pdev)
{
	struct jz_mmc_functions *functions = host->plat->driver_data;

	functions->gpio_deinit(host, pdev);
	functions->msc_deinit(host);
	functions->dma_deinit(host);
}

int controller_register(struct jz_mmc_controller *controller, struct jz_mmc_host *host)
{
	if(controller == NULL)
		return -ENOMEM;

	jz_mmc_gpio_register(&(controller->gpio));
	jz_mmc_msc_register(&(controller->msc));
	jz_mmc_dma_register(&(controller->dma));

	controller->init = controller_init;
	controller->functions.deinit = controller_deinit;
	controller->functions.transmit_data = data_transmit_dma;
	controller->functions.execute_cmd = controller->msc.execute_cmd;
	controller->functions.set_clock = controller->msc.set_clock;
	controller->functions.msc_deinit = controller->msc.deinit;
	controller->functions.gpio_deinit = controller->gpio.deinit;
	controller->functions.dma_deinit = controller->dma.deinit;

	host->plat->driver_data = &(controller->functions);

	// struct jz_mmc_functions *functions = host->plat->driver_data;

//	printk("%s: host->plat->driver_data->set_clock = %x\n", __FUNCTION__, functions->set_clock);

	return 0;
}

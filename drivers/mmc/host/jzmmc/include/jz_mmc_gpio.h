/*
 *  linux/drivers/mmc/host/jz_mmc/gpio/jz_mmc_gpio.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ_MMC_GPIO_H__
#define __JZ_MMC_GPIO_H__

#include "jz_mmc_host.h"
#include <linux/platform_device.h>

#ifndef __JZ_MMC_HOST_H__
#error "!!!!!!!!!!!!!"
#endif

//struct jz_mmc_host test;
//int t = test.dma.len;

struct jz_mmc_gpio {

	int (*init) (struct jz_mmc_host *, struct platform_device *);
	void (*deinit) (struct jz_mmc_host *, struct platform_device *);
};

int jz_mmc_gpio_register(struct jz_mmc_gpio *);

#endif /* __JZ_MMC_GPIO_H__ */


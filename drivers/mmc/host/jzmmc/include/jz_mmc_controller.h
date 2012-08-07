/*
 *  drivers/mmc/host/jz_mmc_controller.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ_MMC_CONTROLLER_H__
#define __JZ_MMC_CONTROLLER_H__

#include "jz_mmc_host.h"
#include "jz_mmc_gpio.h"
#include "jz_mmc_msc.h"
#include "jz_mmc_dma.h"

struct jz_mmc_functions {
	void (*deinit) (struct jz_mmc_host *, struct platform_device *);
	int (*transmit_data) (struct jz_mmc_host *);
	void (*execute_cmd) (struct jz_mmc_host *);
	void (*set_clock) (struct jz_mmc_host *, int);
	void (*msc_deinit) (struct jz_mmc_host *);
	void (*gpio_deinit) (struct jz_mmc_host *, struct platform_device *);
	void (*dma_deinit) (struct jz_mmc_host *);
};

struct jz_mmc_controller {

	struct jz_mmc_msc msc;		// msc
	struct jz_mmc_dma dma;		// dma
	struct jz_mmc_gpio gpio;	// gpio

	int (*init) (struct jz_mmc_controller *, struct jz_mmc_host *,
				struct platform_device *);
	struct jz_mmc_functions functions;
};

int controller_register(struct jz_mmc_controller *, struct jz_mmc_host *);

#endif /* __JZ_MMC_CONTROLLER_H__ */

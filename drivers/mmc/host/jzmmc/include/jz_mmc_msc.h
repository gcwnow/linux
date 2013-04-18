/*
 *  linux/drivers/mmc/host/jz_mmc/msc/jz_mmc_msc.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ_MMC_MSC_H__
#define __JZ_MMC_MSC_H__

struct jz_mmc_host;

struct jz_mmc_msc {
	int (*init) (struct jz_mmc_host *);
	void (*deinit) (struct jz_mmc_host *);
	void (*set_clock) (struct jz_mmc_host *, int);
	void (*execute_cmd) (struct jz_mmc_host *);
};

int jz_mmc_msc_register(struct jz_mmc_msc *msc);

void jz_mmc_data_start(struct jz_mmc_host *host);

void jz_mmc_reset(struct jz_mmc_host *host);

#endif /* __JZ_MMC_MSC_H__ */

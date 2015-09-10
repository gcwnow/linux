/*
 * Platform device support for Jz4770 SoC.
 *
 * Copyright 2010, Software Department III
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>

#include <linux/usb/musb.h>

#include <asm/mach-jz4770/base.h>
#include <asm/mach-jz4770/dma.h>
#include <asm/mach-jz4770/irq.h>

#include "platform.h"


/** MMC/SD/SDIO controllers**/

#define JZ_MSC_PLATFORM_DEV(msc_id)					\
	static struct resource jz4770_msc##msc_id##_resources[] = {		\
		{							\
			.start	= JZ4770_MSC##msc_id##_BASE_ADDR,	\
			.end	= JZ4770_MSC##msc_id##_BASE_ADDR + 0x1000 - 1, \
			.flags	= IORESOURCE_MEM,			\
		},							\
		{							\
			.start	= IRQ_MSC##msc_id,			\
			.end	= IRQ_MSC##msc_id,			\
			.flags	= IORESOURCE_IRQ,			\
		},							\
		{							\
			.start	= DMA_ID_MSC##msc_id,			\
			.end	= DMA_ID_MSC##msc_id,			\
			.flags	= IORESOURCE_DMA,			\
		},							\
	};								\
									\
	static u64 jz4770_msc##msc_id##_dmamask =  ~(u32)0;			\
									\
	struct platform_device jz4770_msc##msc_id##_device = {		\
		.name = "jz-msc",					\
		.id = msc_id,						\
		.dev = {						\
			.dma_mask               = &jz4770_msc##msc_id##_dmamask, \
			.coherent_dma_mask      = 0xffffffff,		\
		},							\
		.num_resources  = ARRAY_SIZE(jz4770_msc##msc_id##_resources), \
		.resource       = jz4770_msc##msc_id##_resources,		\
	};

JZ_MSC_PLATFORM_DEV(0)
JZ_MSC_PLATFORM_DEV(1)
JZ_MSC_PLATFORM_DEV(2)

/* Sound devices */

/* I2S */
static struct resource jz_i2s_resources[] = {
	{
		.start	= JZ4770_AIC_BASE_ADDR,
		.end	= JZ4770_AIC_BASE_ADDR + 0x38 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device jz4770_i2s_device = {
	.name		= "jz4770-i2s",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_i2s_resources),
	.resource	= jz_i2s_resources,
};

/* PCM */
struct platform_device jz4770_pcm_device = {
	.name		= "jz4770-pcm-audio",
	.id		= -1,
};

/* Codec */
static struct resource jz_icdc_resources[] = {
	{
		.start	= JZ4770_AIC_BASE_ADDR + 0xA0,
		.end	= JZ4770_AIC_BASE_ADDR + 0xB0 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device jz4770_icdc_device = {
	.name		= "jz4770-icdc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_icdc_resources),
	.resource	= jz_icdc_resources,
};

/* VPU */

static struct resource jz_vpu_resources[] = {
	{
		.start	= JZ4770_AUX_BASE_ADDR,
		.end	= JZ4770_AUX_BASE_ADDR + 0xFFFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* TCSM0 is 16K in size, other 48K is reserved. */
		.start	= JZ4770_TCSM0_BASE_ADDR,
		.end	= JZ4770_TCSM0_BASE_ADDR + 0xFFFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_VPU,
		.end	= IRQ_VPU,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device jz4770_vpu_device = {
	.name		= "jz-vpu",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_vpu_resources),
	.resource	= jz_vpu_resources,
};

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

#include <asm/mach-jz4770/dma.h>
#include <asm/mach-jz4770/irq.h>

#include "platform.h"

#define JZ4770_AUX_BASE_ADDR	0x132A0000
#define JZ4770_TCSM0_BASE_ADDR	0x132B0000
#define JZ4770_TCSM1_BASE_ADDR	0x132C0000

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

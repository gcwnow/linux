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


/*** LCD controller ***/
static struct resource jz_lcd_resources[] = {
	[0] = {
		.start          = JZ4770_LCD_BASE_ADDR,
		.end            = JZ4770_LCD_BASE_ADDR + 0x13F,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = JZ4770_IPU_BASE_ADDR,
		.end            = JZ4770_IPU_BASE_ADDR + 0x9B,
		.flags          = IORESOURCE_MEM,
	},
	{
		.name           = "tve",
		.start          = JZ4770_LCD_BASE_ADDR + 0x140,
		.end            = JZ4770_LCD_BASE_ADDR + 0x1BF,
		.flags          = IORESOURCE_MEM,
	},
	{
		.name           = "part2",
		.start          = JZ4770_LCD_BASE_ADDR + 0x1C0,
		.end            = JZ4770_LCD_BASE_ADDR + 0x2FF,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_LCD,
		.end            = IRQ_LCD,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_lcd_dmamask = ~(u32)0;

struct platform_device jz4770_lcd_device = {
	.name           = "jz-lcd",
	.id             = 0,
	.dev = {
		.dma_mask               = &jz_lcd_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_lcd_resources),
	.resource       = jz_lcd_resources,
};

/* USB OTG Controller */
struct platform_device jz4770_usb_otg_xceiv_device = {
	.name	= "usb_phy_generic",
	.id	= 0,
};

static struct musb_hdrc_config jz_usb_otg_config = {
	.multipoint	= 1,
/* Max EPs scanned. Driver will decide which EP can be used automatically. */
	.num_eps	= 6,
};

static struct musb_hdrc_platform_data jz_usb_otg_platform_data = {
	.mode           = MUSB_OTG,
	.config		= &jz_usb_otg_config,
};

static struct resource jz_usb_otg_resources[] = {
	[0] = {
		.start		= JZ4770_UDC_BASE_ADDR,
		.end		= JZ4770_UDC_BASE_ADDR + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_OTG,
		.end		= IRQ_OTG,
		.flags		= IORESOURCE_IRQ,
		.name		= "mc",
	},
};

struct platform_device jz4770_usb_otg_device = {
	.name	= "musb-jz",
	.id	= 0,
	.dev = {
		.dma_mask		= &jz4770_usb_otg_device.dev.coherent_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &jz_usb_otg_platform_data,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_otg_resources),
	.resource	= jz_usb_otg_resources,
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

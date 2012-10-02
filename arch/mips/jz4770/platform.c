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
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>

#include <linux/usb/musb.h>

#include <asm/mach-jz4770/dma.h>
#include <asm/mach-jz4770/jz4770aic.h>
#include <asm/mach-jz4770/jz4770aosd.h>
#include <asm/mach-jz4770/jz4770i2c.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770lcdc.h>
#include <asm/mach-jz4770/jz4770msc.h>
#include <asm/mach-jz4770/jz4770otg.h>
#include <asm/mach-jz4770/jz4770sadc.h>
#include <asm/mach-jz4770/jz4770tcu.h>
#include <asm/mach-jz4770/platform.h>


/* OHCI (USB full speed host controller) */
#define UHC_BASE		0xB3430000
static struct resource jz_usb_ohci_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UHC_BASE), // phys addr for ioremap
		.end		= CPHYSADDR(UHC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_UHC,
		.end		= IRQ_UHC,
		.flags		= IORESOURCE_IRQ,
	},
};

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static struct platform_device jz_usb_ohci_device = {
	.name		= "jz-ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_ohci_resources),
	.resource	= jz_usb_ohci_resources,
};

/*** LCD controller ***/
static struct resource jz_lcd_resources[] = {
	[0] = {
		.start          = CPHYSADDR(LCD_BASE),
		.end            = CPHYSADDR(LCD_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_LCD,
		.end            = IRQ_LCD,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_lcd_dmamask = ~(u32)0;

struct platform_device jz_lcd_device = {
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
static struct platform_device jz_usb_otg_xceiv_device = {
	.name	= "nop_usb_xceiv",
	.id	= 0,
};

static struct musb_hdrc_config jz_usb_otg_config = {
	.multipoint	= 1,
	.dyn_fifo	= 0,
	.soft_con	= 1,
	.dma		= 1,
/* Max EPs scanned. Driver will decide which EP can be used automatically. */
	.num_eps	= 6,
};

static struct musb_hdrc_platform_data jz_usb_otg_platform_data = {
	.mode           = MUSB_OTG,
	.config		= &jz_usb_otg_config,
};

static struct resource jz_usb_otg_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UDC_BASE),
		.end		= CPHYSADDR(UDC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_OTG,
		.end		= IRQ_OTG,
		.flags		= IORESOURCE_IRQ,
		.name		= "mc",
	},
};

static u64  usb_otg_dmamask = ~(u32)0;

static struct platform_device jz_usb_otg_device = {
	.name	= "musb-jz",
	.id	= 0,
	.dev = {
		.dma_mask		= &usb_otg_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &jz_usb_otg_platform_data,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_otg_resources),
	.resource	= jz_usb_otg_resources,
};

/** MMC/SD/SDIO controllers**/
#define JZ_MSC_PLATFORM_DEV(msc_id)				\
	static struct resource jz_msc##msc_id##_resources[] = {		\
		{							\
			.start          = CPHYSADDR(MSC##msc_id##_BASE), \
			.end            = CPHYSADDR(MSC##msc_id##_BASE) + 0x1000 - 1, \
			.flags          = IORESOURCE_MEM,		\
		},							\
		{							\
			.start          = IRQ_MSC##msc_id,		\
			.end            = IRQ_MSC##msc_id,		\
			.flags          = IORESOURCE_IRQ,		\
		},							\
		{							\
			.start          = DMA_ID_MSC##msc_id,		\
			.end            = DMA_ID_MSC##msc_id,		\
			.flags          = IORESOURCE_DMA,		\
		},							\
	};								\
									\
	static u64 jz_msc##msc_id##_dmamask =  ~(u32)0;			\
									\
	static struct platform_device jz_msc##msc_id##_device = {	\
		.name = "jz-msc",					\
		.id = msc_id,						\
		.dev = {						\
			.dma_mask               = &jz_msc##msc_id##_dmamask, \
			.coherent_dma_mask      = 0xffffffff,		\
		},							\
		.num_resources  = ARRAY_SIZE(jz_msc##msc_id##_resources), \
		.resource       = jz_msc##msc_id##_resources,		\
	};

JZ_MSC_PLATFORM_DEV(0)
JZ_MSC_PLATFORM_DEV(1)
JZ_MSC_PLATFORM_DEV(2)

static struct platform_device *jz_msc_devices[] __initdata = {
	&jz_msc0_device,
	&jz_msc1_device,
	&jz_msc2_device,
};

int __init jz_add_msc_devices(unsigned int id, struct jz_mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (JZ_MSC_ID_INVALID(id))
		return -EINVAL;

	pdev = jz_msc_devices[id];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

/* Sound devices */

/* I2S */
static struct resource jz_i2s_resources[] = {
	{
		.start	= CPHYSADDR(AIC_BASE),
		.end	= CPHYSADDR(AIC_BASE) + 0x38 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device jz_i2s_device = {
	.name		= "jz4770-i2s",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_i2s_resources),
	.resource	= jz_i2s_resources,
};

/* PCM */
struct platform_device jz_pcm_device = {
	.name		= "jz4770-pcm-audio",
	.id		= -1,
};

/* Codec */
static struct resource jz_icdc_resources[] = {
	{
		.start	= CPHYSADDR(AIC_BASE) + 0xA0,
		.end	= CPHYSADDR(AIC_BASE) + 0xB0 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device jz_icdc_device = {
	.name		= "jz4770-icdc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_icdc_resources),
	.resource	= jz_icdc_resources,
};

/* I2C devices */

static struct resource jz_i2c0_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C_BASE(0)),
		.end            = CPHYSADDR(I2C_BASE(0)) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C0,
		.end            = IRQ_I2C0,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct resource jz_i2c1_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C_BASE(1)),
		.end            = CPHYSADDR(I2C_BASE(1)) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C1,
		.end            = IRQ_I2C1,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct resource jz_i2c2_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C_BASE(2)),
		.end            = CPHYSADDR(I2C_BASE(2)) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C2,
		.end            = IRQ_I2C2,
		.flags          = IORESOURCE_IRQ,
	},
};

static u64 jz_i2c_dmamask =  ~(u32)0;

struct platform_device jz_i2c0_device = {
	.name = "i2c-jz4770",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c0_resources),
	.resource       = jz_i2c0_resources,
};

struct platform_device jz_i2c1_device = {
	.name = "i2c-jz4770",
	.id = 1,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c1_resources),
	.resource       = jz_i2c1_resources,
};

struct platform_device jz_i2c2_device = {
	.name = "i2c-jz4770",
	.id = 2,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c2_resources),
	.resource       = jz_i2c2_resources,
};

/*AOSD*/
static struct resource jz_aosd_resources[] = {
	[0] = {
		.start          = CPHYSADDR(AOSD_BASE),
		.end            = CPHYSADDR(AOSD_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_AOSD,
		.end            = IRQ_AOSD,
		.flags          = IORESOURCE_IRQ,
	},
};

static u64 jz_aosd_dmamask =  ~(u32)0;

static struct platform_device jz_aosd_device = {
	.name = "jz-aosd",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_aosd_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_aosd_resources),
	.resource       = jz_aosd_resources,
};

/* TCU */
static struct resource jz_tcu_resources[] = {
	[0] = {
		.start		= CPHYSADDR(TCU_BASE),
		.end		= CPHYSADDR(TCU_BASE) + 0x1000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_TCU0,
		.end		= IRQ_TCU0,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device jz_tcu_device = {
	.name = "JZ-TCU",
	.id = 0,
	.num_resources = ARRAY_SIZE(jz_tcu_resources),
	.resource = jz_tcu_resources,
};

/* PWM */
static struct platform_device jz_pwm_device = {
	.name = "jz4770-pwm",
	.id   = -1,
};

/* RTC */
static struct platform_device rtc_device = {
	.name		= "jz4770-rtc",
	.id		= -1,
};

/* ADC controller */
static struct resource jz_adc_resources[] = {
	{
		/* Assign only the shared registers to the MFD driver. */
		.start	= CPHYSADDR(SADC_BASE),
		.end	= CPHYSADDR(SADC_BASE) + 0x2F,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_SADC,
		.end	= IRQ_SADC,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= IRQ_SADC_BASE,
		.end	= IRQ_SADC_BASE + SADC_IRQ_NUM - 1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device jz_adc_device = {
	.name		= "jz4770-adc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_adc_resources),
	.resource	= jz_adc_resources,
};

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
	&jz_usb_ohci_device,
	&jz_usb_otg_xceiv_device,
	&jz_usb_otg_device,
	&jz_lcd_device,
	&jz_aosd_device,
	&jz_i2s_device,
	&jz_pcm_device,
	&jz_icdc_device,
	/*
	 * All three I2C devices are listed, but they will only be instantiated
	 * if the corresponding platform data is set by board_i2c_init().
	 */
	&jz_i2c0_device,
	&jz_i2c1_device,
	&jz_i2c2_device,
	&jz_tcu_device,
	&jz_pwm_device,
	&jz_adc_device,
	&rtc_device,
};

static int __init jz_platform_init(void)
{
	int ret;

	board_i2c_init();
	board_pdata_init();
	ret = platform_add_devices(jz_platform_devices,
				   ARRAY_SIZE(jz_platform_devices));
	board_devices_init();

	printk("jz_platform_init\n");
	board_msc_init();
	return ret;
}

arch_initcall(jz_platform_init);

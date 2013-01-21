/*
 * board-gcw0.c  -  GCW Zero: JZ4770-based handheld game console
 *
 * File based on Pisces board definition.
 * Copyright (C) 2006-2008, Ingenic Semiconductor Inc.
 * Original author: <jlwei@ingenic.cn>
 *
 * GCW Zero specific changes:
 * Copyright (C) 2012, Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <linux/mmc/host.h>
#include <linux/act8600_power.h>
#include <linux/power/jz4770-battery.h>
#include <media/radio-rda5807.h>
#include <video/jzpanel.h>
#include <video/panel-nt39016.h>

#include <asm/mach-jz4770/board-gcw0.h>
#include <asm/mach-jz4770/jz4770_fb.h>
#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/jz4770i2c.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770misc.h>
#include <asm/mach-jz4770/jz4770msc.h>
#include <asm/mach-jz4770/platform.h>

#include "clock.h"

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

/*
 *  config_gpio_on_sleep config all gpio pins when sleep.
 */
struct gpio_sleep_status {
	unsigned int input;
	unsigned int input_pull;
	unsigned int input_no_pull;
	unsigned int output;
	unsigned int output_high;
	unsigned int output_low;
	unsigned int no_operation;
};

void config_gpio_on_sleep(void)
{
	int i = 0;
	struct gpio_sleep_status gpio_sleep_st[] = {
		/* GPA */
		{
			.input_pull = BIT31 | BIT27 |
			BITS_H2L(29,28) | /* NC pin */
			BITS_H2L(19,0), /* NAND: SD0~SD15 */

			.output_high = BIT21 | BIT22 | BIT23 | BIT24 | BIT25 | BIT26, /* NAND: CS1~CS6 */
			.output_low = 0x0,
#ifndef CONFIG_JZ_SYSTEM_AT_CARD
			.no_operation = 0x0,
#else
			.no_operation = BITS_H2L(23, 18),
#endif
		},

		/* GPB */
		{
			.input_pull = BIT30 | BIT27 | BIT26 | BIT25 | BITS_H2L(24,22) | BIT20 |
			BITS_H2L(19,0), /* SA0~SA5 */

			.output_high = BIT29,
			.output_low = BIT31 | BIT28 | BIT21,
#ifndef CONFIG_JZ_SYSTEM_AT_CARD
			.no_operation = 0x0,
#else
			.no_operation = BIT0,
#endif
		},

		/* GPC */
		{
			.input_pull = BITS_H2L(31,28),
			.output_high = 0x0,
			.output_low = BITS_H2L(27,0),
			.no_operation = 0x0,
		},

		/* GPD */
		{
			.input_pull = BITS_H2L(29,26) | BITS_H2L(19,14) | BITS_H2L(13,12) || BITS_H2L(10,0) | BIT11,  // bit11 temporary input_pull
			.output_high = 0x0,
			.output_low =  BITS_H2L(25,20), // | BIT11,
			.no_operation = 0x0,
		},

		/* GPE */
		{
			.input_pull = BITS_H2L(18,11) | BITS_H2L(8,3) | BIT0,
			.output_high = BIT9,
			.output_low = BITS_H2L(29,20) | BIT10 | BIT1 | BIT2,
			.no_operation = 0x0,
		},

		/* GPF */
		{
			.input_pull = BIT11 | BITS_H2L(8,4) | BITS_H2L(2,0),
			.output_high = BIT9,
			.output_low = BIT3,
			.no_operation = 0x0,
		},
	};

	for (i = 0; i < 6; i++) {
		gpio_sleep_st[i].input_pull &= ~gpio_sleep_st[i].no_operation;
		gpio_sleep_st[i].output_high &= ~gpio_sleep_st[i].no_operation;
		gpio_sleep_st[i].output_low &= ~gpio_sleep_st[i].no_operation;
		gpio_sleep_st[i].input_no_pull = 0xffffffff &
			~(gpio_sleep_st[i].input_pull |
			  gpio_sleep_st[i].output_high |
			  gpio_sleep_st[i].output_low) &
			~gpio_sleep_st[i].no_operation;

		gpio_sleep_st[i].input = gpio_sleep_st[i].input_pull | gpio_sleep_st[i].input_no_pull;
		gpio_sleep_st[i].output = gpio_sleep_st[i].output_high | gpio_sleep_st[i].output_low;

		/* all as gpio, except interrupt pins(see @wakeup_key_setup()) */
		REG_GPIO_PXINTC(i) =  (0xffffffff & ~gpio_sleep_st[i].no_operation);
		REG_GPIO_PXMASKS(i) =  (0xffffffff & ~gpio_sleep_st[i].no_operation);
		/* input */
		REG_GPIO_PXPAT1S(i) =  gpio_sleep_st[i].input;
		/* pull */
		REG_GPIO_PXPENC(i) = gpio_sleep_st[i].input_pull;
		/* no_pull */
		REG_GPIO_PXPENS(i) =  gpio_sleep_st[i].input_no_pull;

		/* output */
		REG_GPIO_PXPAT1C(i) =  gpio_sleep_st[i].output;
		REG_GPIO_PXPENS(i)  = gpio_sleep_st[i].output; /* disable pull */
		/* high */
		REG_GPIO_PXPAT0S(i) = gpio_sleep_st[i].output_high;
		/* low */
		REG_GPIO_PXPAT0C(i) = gpio_sleep_st[i].output_low;
	}
}

struct wakeup_key_s {
	int gpio;       /* gpio pin number */
	int active_low; /* the key interrupt pin is low voltage
                           or fall edge acitve */
};

/* add wakeup keys here */
static struct wakeup_key_s wakeup_key[] = {
	{
		.gpio = GPIO_POWER_ON,
		.active_low = ACTIVE_LOW_WAKE_UP,
	},
/*
#ifndef CONFIG_JZ_SYSTEM_AT_CARD
	{
		.gpio = GPIO_SD0_CD_N,
		.active_low = ACTIVE_LOW_MSC0_CD,
	},
#endif
	{
		.gpio = GPIO_SD1_CD_N,
		.active_low = ACTIVE_LOW_MSC1_CD,
	},
*/
};

static void wakeup_key_setup(void)
{
	int i;
	int num = sizeof(wakeup_key) / sizeof(wakeup_key[0]);

	for(i = 0; i < num; i++) {
		if(wakeup_key[i].active_low)
			__gpio_as_irq_fall_edge(wakeup_key[i].gpio);
		else
			__gpio_as_irq_rise_edge(wakeup_key[i].gpio);

		__gpio_ack_irq(wakeup_key[i].gpio);
		__gpio_unmask_irq(wakeup_key[i].gpio);
		__intc_unmask_irq(IRQ_GPIO0 - (wakeup_key[i].gpio/32));  /* unmask IRQ_GPIOn */
	}
}


/* NOTES:
 * 1: Pins that are floated (NC) should be set as input and pull-enable.
 * 2: Pins that are pull-up or pull-down by outside should be set as input
 *    and pull-disable.
 * 3: Pins that are connected to a chip except sdram and nand flash
 *    should be set as input and pull-disable, too.
 */
void jz_board_do_sleep(unsigned long *ptr)
{
	unsigned char i;

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("run int:%x mask:%x pat1:%x pat0:%x pen:%x flg:%x\n",        \
			REG_GPIO_PXINT(i),REG_GPIO_PXMASK(i),REG_GPIO_PXPAT1(i),REG_GPIO_PXPAT0(i), \
			REG_GPIO_PXPEN(i),REG_GPIO_PXFLG(i));
	}

        /* Save GPIO registers */
	for(i = 1; i < GPIO_PORT_NUM; i++) {
		*ptr++ = REG_GPIO_PXINT(i);
		*ptr++ = REG_GPIO_PXMASK(i);
		*ptr++ = REG_GPIO_PXPAT0(i);
		*ptr++ = REG_GPIO_PXPAT1(i);
		*ptr++ = REG_GPIO_PXFLG(i);
		*ptr++ = REG_GPIO_PXPEN(i);
	}

        /*
         * Set all pins to pull-disable, and set all pins as input except
         * sdram and the pins which can be used as CS1_N to CS4_N for chip select.
         */
	config_gpio_on_sleep();

        /*
	 * Set proper status for GPC21 to GPC24 which can be used as CS1_N to CS4_N.
	 * Keep the pins' function used for chip select(CS) here according to your
         * system to avoid chip select crashing with sdram when resuming from sleep mode.
         */

	/*
         * If you must set some GPIOs as output to high level or low level,
         * you can set them here, using:
         * __gpio_as_output(n);
         * __gpio_set_pin(n); or  __gpio_clear_pin(n);
	 */

	if (!console_suspend_enabled)
		__gpio_as_uart2();

#if 0
        /* Keep uart function for printing debug message */
	__gpio_as_uart0();
	__gpio_as_uart1();
	__gpio_as_uart2();
//	__gpio_as_uart3();

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		printk("GP%d: data:0x%08x pin:0x%08x fun:0x%08x sel:0x%08x dir:0x%08x pull:0x%08x msk:0x%08x trg:0x%08x\n",
		       i, REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i),
		       REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}
#endif
	wakeup_key_setup();
}

void jz_board_do_resume(unsigned long *ptr)
{
	unsigned char i;

	/* Restore GPIO registers */
		for(i = 1; i < GPIO_PORT_NUM; i++) {
		 REG_GPIO_PXINTS(i) = *ptr;
		 REG_GPIO_PXINTC(i) = ~(*ptr++);

		 REG_GPIO_PXMASKS(i) = *ptr;
		 REG_GPIO_PXMASKC(i) = ~(*ptr++);

		 REG_GPIO_PXPAT0S(i) = *ptr;
		 REG_GPIO_PXPAT0C(i) = ~(*ptr++);

		 REG_GPIO_PXPAT1S(i) = *ptr;
		 REG_GPIO_PXPAT1C(i) = ~(*ptr++);

//		 REG_GPIO_PXFLGS(i)=*ptr;
		 REG_GPIO_PXFLGC(i)=~(*ptr++);

		 REG_GPIO_PXPENS(i)=*ptr;
		 REG_GPIO_PXPENC(i)=~(*ptr++);
	}

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("run int:%x mask:%x pat1:%x pat0:%x pen:%x flg:%x\n",        \
			REG_GPIO_PXINT(i),REG_GPIO_PXMASK(i),REG_GPIO_PXPAT1(i),REG_GPIO_PXPAT0(i), \
			REG_GPIO_PXPEN(i),REG_GPIO_PXFLG(i));
	}
}

extern void (*jz_timer_callback)(void);
extern int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);

static void dancing(void)
{
	static unsigned char slash[] = "\\|/-";
//	static volatile unsigned char *p = (unsigned char *)0xb6000058;
	static volatile unsigned char *p = (unsigned char *)0xb6000016;
	static unsigned int count = 0;
	*p = slash[count++];
	count &= 3;
}


/* Video */

#define GPIO_PANEL_SOMETHING	GPF(0)

static int __devinit gcw0_panel_init(void **out_panel,
				     struct device *dev, void *panel_pdata)
{
	int ret;

	ret = nt39016_panel_ops.init(out_panel, dev, panel_pdata);
	if (ret)
		return ret;

	ret = devm_gpio_request(dev, GPIO_PANEL_SOMETHING, "LCD panel unknown");
	if (ret) {
		dev_err(dev,
			"Failed to request LCD panel unknown pin: %d\n", ret);
		return ret;
	}

	gpio_direction_output(GPIO_PANEL_SOMETHING, 1);

	return 0;
}

static void __devexit gcw0_panel_exit(void *panel)
{
	nt39016_panel_ops.exit(panel);
}

static void gcw0_panel_enable(void *panel)
{
	act8600_output_enable(6, true);

	nt39016_panel_ops.enable(panel);
}

static void gcw0_panel_disable(void *panel)
{
	nt39016_panel_ops.disable(panel);

	act8600_output_enable(6, false);
}

static struct nt39016_platform_data gcw0_panel_pdata = {
	.gpio_reset		= GPE(2),
	.gpio_clock		= GPE(15),
	.gpio_enable		= GPE(16),
	.gpio_data		= GPE(17),
};

static struct panel_ops gcw0_panel_ops = {
	.init		= gcw0_panel_init,
	.exit		= gcw0_panel_exit,
	.enable		= gcw0_panel_enable,
	.disable	= gcw0_panel_disable,
};

static struct jzfb_platform_data gcw0_fb_pdata = {
	.panel_ops		= &gcw0_panel_ops,
	.panel_pdata		= &gcw0_panel_pdata,
};


/* Buttons */

static struct gpio_keys_button gcw0_buttons[] = {
	/* D-pad up */ {
		.gpio			= GPE(21),
		.active_low		= 1,
		.code			= KEY_UP,
		.debounce_interval	= 10,
	},
	/* D-pad down */ {
		.gpio			= GPE(25),
		.active_low		= 1,
		.code			= KEY_DOWN,
		.debounce_interval	= 10,
	},
	/* D-pad left */ {
		.gpio			= GPE(23),
		.active_low		= 1,
		.code			= KEY_LEFT,
		.debounce_interval	= 10,
	},
	/* D-pad right */ {
		.gpio			= GPE(24),
		.active_low		= 1,
		.code			= KEY_RIGHT,
		.debounce_interval	= 10,
	},
	/* A button */ {
		.gpio			= GPE(29),
		.active_low		= 1,
		.code			= KEY_LEFTCTRL,
		.debounce_interval	= 10,
	},
	/* B button */ {
		.gpio			= GPE(20),
		.active_low		= 1,
		.code			= KEY_LEFTALT,
		.debounce_interval	= 10,
	},
	/* Top button (labeled Y, should be X) */ {
		.gpio			= GPE(27),
		.active_low		= 1,
		.code			= KEY_SPACE,
		.debounce_interval	= 10,
	},
	/* Left button (labeled X, should be Y) */ {
		.gpio			= GPE(28),
		.active_low		= 1,
		.code			= KEY_LEFTSHIFT,
		.debounce_interval	= 10,
	},
	/* Left shoulder button */ {
		.gpio			= GPB(20),
		.active_low		= 1,
		.code			= KEY_TAB,
		.debounce_interval	= 10,
	},
	/* Right shoulder button */ {
		.gpio			= GPE(26),
		.active_low		= 1,
		.code			= KEY_BACKSPACE,
		.debounce_interval	= 10,
	},
	/* START button */ {
		.gpio			= GPB(21),
		.active_low		= 1,
		.code			= KEY_ENTER,
		.debounce_interval	= 10,
	},
	/* SELECT button */ {
		.gpio			= GPD(18),
		/* This is the only button that is active high,
		 * since it doubles as BOOT_SEL1.
		 */
		.active_low		= 0,
		.code			= KEY_ESC,
		.debounce_interval	= 10,
	},
	/* POWER slider */ {
		.gpio			= GPA(30),
		.active_low		= 1,
		.code			= KEY_POWER,
		.debounce_interval	= 10,
		.wakeup			= 1,
	},
	/* POWER hold */ {
		.gpio			= GPF(11),
		.active_low		= 1,
		.code			= KEY_PAUSE,
		.debounce_interval	= 10,
	},
};

static struct gpio_keys_platform_data gcw0_gpio_keys_pdata = {
	.buttons = gcw0_buttons,
	.nbuttons = ARRAY_SIZE(gcw0_buttons),
	.rep = 1,
};

static struct platform_device gcw0_gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &gcw0_gpio_keys_pdata,
	},
};


/* SD cards */

/* SD0 is probably permanently powered, since it is bootable. */
#define GPIO_SD1_VCC_EN_N	GPE(9)
#define GPIO_SD1_CD_N		GPB(2)

static void gcw_internal_sd_gpio_init(struct device *dev)
{
	__gpio_as_msc0_boot();
}

static void gcw_internal_sd_power_on(struct device *dev)
{
}

static void gcw_internal_sd_power_off(struct device *dev)
{
}

struct jz_mmc_platform_data gcw_internal_sd_data = {
	.support_sdio   = 0,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init           = gcw_internal_sd_gpio_init,
	.power_on       = gcw_internal_sd_power_on,
	.power_off      = gcw_internal_sd_power_off,
	.max_bus_width  = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
			  MMC_CAP_4_BIT_DATA,
	.bus_width      = 4,
};

static void gcw_external_sd_gpio_init(struct device *dev)
{
	__gpio_as_msc1_4bit();
	__gpio_as_output1(GPIO_SD1_VCC_EN_N); /* poweroff */
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void gcw_external_sd_power_on(struct device *dev)
{
	__gpio_as_output0(GPIO_SD1_VCC_EN_N);
}

static void gcw_external_sd_power_off(struct device *dev)
{
	__gpio_as_output1(GPIO_SD1_VCC_EN_N);
}

static unsigned int gcw_external_sd_status(struct device *dev)
{
	return !__gpio_get_pin(GPIO_SD1_CD_N);
}

static void gcw_external_sd_plug_change(int state)
{
	if (state == CARD_INSERTED)	/* wait for remove */
		__gpio_as_irq_high_level(GPIO_SD1_CD_N);
	else				/* wait for insert */
		__gpio_as_irq_low_level(GPIO_SD1_CD_N);
}

struct jz_mmc_platform_data gcw_external_sd_data = {
	.support_sdio   = 0,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= IRQ_GPIO_0 + GPIO_SD1_CD_N,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = gcw_external_sd_gpio_init,
	.power_on       = gcw_external_sd_power_on,
	.power_off      = gcw_external_sd_power_off,
	.status		= gcw_external_sd_status,
	.plug_change	= gcw_external_sd_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.bus_width      = 4,
};

void __init board_msc_init(void)
{
	jz_add_msc_devices(0, &gcw_internal_sd_data);
	jz_add_msc_devices(1, &gcw_external_sd_data);
}


static void f4770_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4770/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/* SELECT button */
	__gpio_disable_pull(GPD(18));

	/* WiFi enable (low active) */
	__gpio_as_output0(GPF(10));

	/* USB plug inserted (high active) */
	__gpio_disable_pull(GPB(5));

	/* TODO(MtH): Figure out the purpose of these pins. */
	__gpio_as_output0(GPB(28));
	__gpio_as_output1(GPE(8));
	__gpio_disable_pull(GPF(5));
}


/* FM radio receiver */

static struct rda5807_platform_data gcw0_rda5807_pdata = {
	.input_flags		= RDA5807_INPUT_LNA_WC_25 | RDA5807_LNA_PORT_P,
	.output_flags		= RDA5807_OUTPUT_AUDIO_ANALOG,
};


/* Power Management Unit */

static struct act8600_outputs_t act8600_outputs[] = {
	{ 4, 0x57, true  }, /* USB OTG: 5.3V */
	{ 5, 0x31, true  }, /* AVD:     2.5V */
	{ 6, 0x39, false }, /* LCD:     3.3V */
	{ 7, 0x39, true  }, /* generic: 3.3V */
	{ 8, 0x24, true  }, /* generic: 1.8V */
};

static struct act8600_platform_pdata_t act8600_platform_pdata = {
        .outputs = act8600_outputs,
        .nr_outputs = ARRAY_SIZE(act8600_outputs),
};


/* Battery */

static struct jz_battery_platform_data gcw0_battery_pdata = {
	.gpio_charge = -1,
	//.gpio_charge_active_low = 0,
	.info = {
		.name = "battery",
		.technology = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design = 5700000,
		.voltage_min_design = 4600000,
	},
};


/* I2C devices */

static struct i2c_board_info gcw0_i2c0_devs[] __initdata = {
	{
		.type		= "radio-rda5807",
		.addr		= RDA5807_I2C_ADDR,
		.platform_data	= &gcw0_rda5807_pdata,
	},
};

static struct i2c_board_info gcw0_i2c1_devs[] __initdata = {
	/* the g-sensor is on this bus, but we don't have a driver for it */
};

static struct i2c_board_info gcw0_i2c2_devs[] __initdata = {
};

static struct i2c_board_info gcw0_i2c3_devs[] __initdata = {
	{
		.type		= ACT8600_NAME,
		.addr		= ACT8600_I2C_ADDR,
		.platform_data	= &act8600_platform_pdata,
	},
};

static struct i2c_board_info gcw0_i2c4_devs[] __initdata = {
	/* the IT6610 is on this bus, but we don't have a driver for it */
};

/* I2C busses */

static struct i2c_jz4770_platform_data gcw0_i2c0_platform_data __initdata = {
	.use_dma		= false,
};

static struct i2c_jz4770_platform_data gcw0_i2c1_platform_data __initdata = {
	.use_dma		= false,
};

static struct i2c_jz4770_platform_data gcw0_i2c2_platform_data __initdata = {
	.use_dma		= false,
};

#if !defined(CONFIG_I2C_JZ4770)

static struct i2c_gpio_platform_data gcw0_i2c0_gpio_data = {
	.sda_pin		= GPD(30),
	.scl_pin		= GPD(31),
	.udelay			= 2, /* 250 kHz */
};

static struct platform_device gcw0_i2c0_gpio_device = {
	.name			= "i2c-gpio",
	.id			= 0,
	.dev			= {
		.platform_data = &gcw0_i2c0_gpio_data,
	},
};

static struct i2c_gpio_platform_data gcw0_i2c1_gpio_data = {
	.sda_pin		= GPE(30),
	.scl_pin		= GPE(31),
	.udelay			= 2, /* 250 kHz */
};

static struct platform_device gcw0_i2c1_gpio_device = {
	.name			= "i2c-gpio",
	.id			= 1,
	.dev			= {
		.platform_data = &gcw0_i2c1_gpio_data,
	},
};

static struct i2c_gpio_platform_data gcw0_i2c2_gpio_data = {
	.sda_pin		= GPF(16),
	.scl_pin		= GPF(17),
	.udelay			= 2, /* 250 kHz */
};

static struct platform_device gcw0_i2c2_gpio_device = {
	.name			= "i2c-gpio",
	.id			= 2,
	.dev			= {
		.platform_data = &gcw0_i2c2_gpio_data,
	},
};

#endif

static struct i2c_gpio_platform_data gcw0_i2c3_gpio_data = {
	.sda_pin		= GPD(5),
	.scl_pin		= GPD(4),
	.udelay			= 2, /* 250 kHz */
};

static struct platform_device gcw0_i2c3_gpio_device = {
	.name			= "i2c-gpio",
	.id			= 3,
	.dev			= {
		.platform_data = &gcw0_i2c3_gpio_data,
	},
};

static struct i2c_gpio_platform_data gcw0_i2c4_gpio_data = {
	.sda_pin		= GPD(6),
	.scl_pin		= GPD(7),
	.udelay			= 5, /* 100 kHz */
};

static struct platform_device gcw0_i2c4_gpio_device = {
	.name			= "i2c-gpio",
	.id			= 4,
	.dev			= {
		.platform_data = &gcw0_i2c4_gpio_data,
	},
};


/* LCD backlight */

static struct platform_pwm_backlight_data gcw0_backlight_pdata = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 145,
	.pwm_period_ns = 40000, /* 25 kHz: outside human hearing range */
};

static struct platform_device gcw0_backlight_device = {
	.name = "pwm-backlight",
	.id = -1,
	.dev = {
		.platform_data = &gcw0_backlight_pdata,
	},
};


/* Audio */

static struct platform_device gcw0_audio_device = {
	.name = "gcw0-audio",
	.id = -1,
};


struct jz_clk_board_data jz_clk_bdata = {
	.ext_rate	=   12000000,
	.rtc_rate	=      32768,
	.pll1_rate	=  192000000,
};


/* Device registration */

static struct platform_device *jz_platform_devices[] __initdata = {
#if !defined(CONFIG_I2C_JZ4770)
	&gcw0_i2c0_gpio_device,
	&gcw0_i2c1_gpio_device,
	&gcw0_i2c2_gpio_device,
#endif
	&gcw0_i2c3_gpio_device,
	&gcw0_i2c4_gpio_device,
	&gcw0_gpio_keys_device,
	&gcw0_backlight_device,
	&gcw0_audio_device,
};

void __init board_pdata_init(void)
{
	jz_lcd_device.dev.platform_data = &gcw0_fb_pdata;
	jz_adc_device.dev.platform_data = &gcw0_battery_pdata;
}

void __init board_devices_init(void)
{
	platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));
}


void __init board_i2c_init(void) {
	jz_i2c0_device.dev.platform_data = &gcw0_i2c0_platform_data;
	jz_i2c1_device.dev.platform_data = &gcw0_i2c1_platform_data;
	jz_i2c2_device.dev.platform_data = &gcw0_i2c2_platform_data;

	i2c_register_board_info(0, gcw0_i2c0_devs, ARRAY_SIZE(gcw0_i2c0_devs));
	i2c_register_board_info(1, gcw0_i2c1_devs, ARRAY_SIZE(gcw0_i2c1_devs));
	i2c_register_board_info(2, gcw0_i2c2_devs, ARRAY_SIZE(gcw0_i2c2_devs));
	i2c_register_board_info(3, gcw0_i2c3_devs, ARRAY_SIZE(gcw0_i2c3_devs));
	i2c_register_board_info(4, gcw0_i2c4_devs, ARRAY_SIZE(gcw0_i2c4_devs));
}

void __init jz_board_setup(void)
{

	printk("JZ4770 GCW0 board setup\n");
//	jz_restart(NULL);
	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = f4770_timer_callback;
}

#if defined(CONFIG_VIVANTE_GPU_GC860) || defined(CONFIG_VIVANTE_GPU_GC860_MODULE)
unsigned long plat_do_mmap_pgoff(struct file *file, unsigned long addr,
				 unsigned long len, unsigned long prot,
				 unsigned long flags, unsigned long pgoff)
{
	return do_mmap_pgoff(file, addr, len, prot, flags, pgoff);
}
EXPORT_SYMBOL(plat_do_mmap_pgoff);

int plat_do_munmap(struct mm_struct *mm, unsigned long start, size_t len)
{
	return do_munmap(mm, start, len);
}
EXPORT_SYMBOL(plat_do_munmap);
#endif

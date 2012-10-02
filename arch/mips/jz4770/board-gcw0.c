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
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>
#include <linux/platform_device.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <linux/mmc/host.h>
#include <linux/act8600_power.h>
#include <linux/platform_data/jz4770_mmc.h>
#include <linux/platform_data/linkdev.h>
#include <linux/platform_data/mxc6225.h>
#include <linux/platform_data/usb-musb-jz4770.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/power/gpio-charger.h>
#include <linux/power/jz4770-battery.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-regulator.h>
#include <linux/usb/musb.h>
#include <sound/jz4770.h>
#include <video/jzpanel.h>
#include <video/panel-nt39016.h>
#include <video/platform_lcd.h>

#include <asm/mach-jz4770/board-gcw0.h>
#include <asm/mach-jz4770/gpio.h>
#include <asm/mach-jz4770/jz4770misc.h>

#include "platform.h"


/* SD cards */

static struct jz_mmc_platform_data gcw_internal_sd_data = {
	.support_sdio		= 0,
	.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34,
	.bus_width		= 4,
	.gpio_card_detect	= -1,
	.gpio_read_only		= -1,
	.gpio_power		= -1,
	.nonremovable		= 1,
};

static struct jz_mmc_platform_data gcw_external_sd_data = {
	.support_sdio		= 0,
	.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34,
	.bus_width		= 4,
	.gpio_card_detect	= JZ_GPIO_PORTB(2),
	.card_detect_active_low	= 1,
	.gpio_read_only		= -1,
	.gpio_power		= JZ_GPIO_PORTE(9),
	.power_active_low	= 1,
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


/* Charger */

#define GPIO_DC_CHARGER		JZ_GPIO_PORTF(5)
#define GPIO_USB_CHARGER	JZ_GPIO_PORTB(5)

static char *gcw0_batteries[] = {
	"battery",
};

static struct gpio_charger_platform_data gcw0_dc_charger_pdata = {
	.name = "dc",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.gpio = GPIO_DC_CHARGER,
	.supplied_to = gcw0_batteries,
	.num_supplicants = ARRAY_SIZE(gcw0_batteries),
};

static struct platform_device gcw0_dc_charger_device = {
	.name = "gpio-charger",
	.id = 0,
	.dev = {
		.platform_data = &gcw0_dc_charger_pdata,
	},
};

static struct gpio_charger_platform_data gcw0_usb_charger_pdata = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.gpio = GPIO_USB_CHARGER,
	.supplied_to = gcw0_batteries,
	.num_supplicants = ARRAY_SIZE(gcw0_batteries),
};

static struct platform_device gcw0_usb_charger_device = {
	.name = "gpio-charger",
	.id = 1,
	.dev = {
		.platform_data = &gcw0_usb_charger_pdata,
	},
};


/* USB 1.1 Host (OHCI) */

static struct regulator_consumer_supply gcw0_internal_usb_regulator_consumer =
	REGULATOR_SUPPLY("vrfkill", "rfkill-regulator.0");

static struct regulator_init_data gcw0_internal_usb_regulator_init_data = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &gcw0_internal_usb_regulator_consumer,
	.constraints = {
		.name = "USB power",
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config gcw0_internal_usb_regulator_data = {
	.supply_name = "USB power",
	.microvolts = 3300000,
	.gpio = JZ_GPIO_PORTF(10),
	.init_data = &gcw0_internal_usb_regulator_init_data,
};

static struct platform_device gcw0_internal_usb_regulator_device = {
	.name = "reg-fixed-voltage",
	.id = -1,
	.dev = {
		.platform_data = &gcw0_internal_usb_regulator_data,
	}
};


/* USB OTG (musb) */

#define GPIO_USB_OTG_ID_PIN	JZ_GPIO_PORTF(18)

static struct jz_otg_board_data gcw0_otg_board_data = {
	.gpio_id_pin = GPIO_USB_OTG_ID_PIN,
	.gpio_id_debounce_ms = 500,
};


/* LCD panel */

static struct platform_device gcw0_lcd_device;
static void *gcw0_lcd_panel;

static struct nt39016_platform_data gcw0_panel_pdata = {
	.gpio_reset		= JZ_GPIO_PORTE(2),
	.gpio_clock		= JZ_GPIO_PORTE(15),
	.gpio_enable		= JZ_GPIO_PORTE(16),
	.gpio_data		= JZ_GPIO_PORTE(17),
};

#define GPIO_PANEL_SOMETHING	JZ_GPIO_PORTF(0)

static int gcw0_lcd_probe(struct plat_lcd_data *pdata)
{
	int ret;

	struct device *dev = &gcw0_lcd_device.dev;

	ret = nt39016_panel_ops.init(&gcw0_lcd_panel, dev, &gcw0_panel_pdata);
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

static void gcw0_lcd_set_power(struct plat_lcd_data *pdata, unsigned int power)
{
	if (power) {
		act8600_output_enable(6, true);
		nt39016_panel_ops.enable(gcw0_lcd_panel);
	} else {
		nt39016_panel_ops.disable(gcw0_lcd_panel);
		act8600_output_enable(6, false);
	}
}

static struct plat_lcd_data gcw0_lcd_pdata = {
	.probe = gcw0_lcd_probe,
	.set_power = gcw0_lcd_set_power,
};

static struct platform_device gcw0_lcd_device = {
	.name = "platform-lcd",
	.dev = {
		.platform_data = &gcw0_lcd_pdata,
		.parent = &jz4770_lcd_device.dev,
	},
};


/* Audio */

static struct jz4770_icdc_platform_data gcw0_icdc_pdata = {
	.mic_mode = JZ4770_MIC_1,
};

static struct platform_device gcw0_audio_device = {
	.name = "gcw0-audio",
	.id = -1,
};


static struct rfkill_regulator_platform_data gcw0_rfkill_pdata = {
	.name = "gcw0-wifi",
	.type = RFKILL_TYPE_WLAN,
};

static struct platform_device gcw0_rfkill_device = {
	.name = "rfkill-regulator",
	.id = 0,
	.dev = {
		.platform_data = &gcw0_rfkill_pdata,
	},
};

static const char * gcw0_joystick_gpiokeys_whitelist[] = {
	"evdev",
};

static const struct linkdev_pdata_device_info gcw0_joystick_devices[] = {
	{
		.name = "analog joystick",
	},
	{
		.name = "board:gpio-keys",
		.handlers_whitelist = gcw0_joystick_gpiokeys_whitelist,
		.nb_handlers = ARRAY_SIZE(gcw0_joystick_gpiokeys_whitelist),
	},
};

static const struct linkdev_pdata_key_map gcw0_key_map[] = {
	{
		.code = KEY_UP,
		.event = {
			.type = EV_ABS,
			.code = ABS_HAT0Y,
			.value = -1,
		},
	},
	{
		.code = KEY_DOWN,
		.event = {
			.type = EV_ABS,
			.code = ABS_HAT0Y,
			.value = 1,
		}
	},
	{
		.code = KEY_LEFT,
		.event = {
			.type = EV_ABS,
			.code = ABS_HAT0X,
			.value = -1,
		},
	},
	{
		.code = KEY_RIGHT,
		.event = {
			.type = EV_ABS,
			.code = ABS_HAT0X,
			.value = 1,
		},
	},
	{
		.code = KEY_LEFTCTRL,
		.event.code = BTN_EAST,
	},
	{
		.code = KEY_LEFTALT,
		.event.code = BTN_SOUTH,
	},
	{
		.code = KEY_LEFTSHIFT,
		.event.code = BTN_WEST,
	},
	{
		.code = KEY_SPACE,
		.event.code = BTN_NORTH,
	},
	{
		.code = KEY_ENTER,
		.event.code = BTN_START,
	},
	{
		.code = KEY_ESC,
		.event.code = BTN_SELECT,
	},
	{
		.code = KEY_TAB,
		.event.code = BTN_THUMBL,
	},
	{
		.code = KEY_BACKSPACE,
		.event.code = BTN_THUMBR,
	},
};

static const struct linkdev_pdata_abs_map gcw0_abs_map[] = {
	{
		.name = "analog joystick",
		.axis = ABS_X,
		.axis_dest = ABS_X,
	},
	{
		.name = "analog joystick",
		.axis = ABS_Y,
		.axis_dest = ABS_Y,
	},
	{
		.name = "gpio-keys",
		.axis = ABS_HAT0X,
		.axis_dest = ABS_HAT0X,
	},
	{
		.name = "gpio-keys",
		.axis = ABS_HAT0Y,
		.axis_dest = ABS_HAT0Y,
	},
};

static struct linkdev_platform_data gcw0_joystick_pdata = {
	/* This specific name informs SDL about the composition of the joystick */
	.name = "linkdev device (Analog 2-axis 8-button 2-hat)",
	.devices = gcw0_joystick_devices,
	.nb_devices = ARRAY_SIZE(gcw0_joystick_devices),
	.key_map = gcw0_key_map,
	.key_map_size = ARRAY_SIZE(gcw0_key_map),
	.abs_map = gcw0_abs_map,
	.abs_map_size = ARRAY_SIZE(gcw0_abs_map),
};

/* GCW0 Input driver */
static struct platform_device gcw0_joystick_device = {
	.name = "linkdev",
	.id = -1,
	.dev = {
		.platform_data = &gcw0_joystick_pdata,
	},
};


/* Device registration */

static struct platform_device *jz_platform_devices[] __initdata = {
	&gcw0_internal_usb_regulator_device,
	&jz4770_usb_ohci_device,
	&jz4770_usb_otg_xceiv_device,
	&jz4770_usb_otg_device,
	&jz4770_lcd_device,
	&jz4770_i2s_device,
	&jz4770_pcm_device,
	&jz4770_icdc_device,
	&jz4770_adc_device,
	&jz4770_rtc_device,
	&gcw0_lcd_device,
	&gcw0_audio_device,
	&jz4770_msc0_device,
	&jz4770_msc1_device,
	&gcw0_dc_charger_device,
	&gcw0_usb_charger_device,
	&jz4770_vpu_device,
	&gcw0_rfkill_device,
	&gcw0_joystick_device,
	&jz4770_wdt_device,
};

static int __init gcw0_init_platform_devices(void)
{
	struct musb_hdrc_platform_data *otg_platform_data =
			jz4770_usb_otg_device.dev.platform_data;
	otg_platform_data->board_data = &gcw0_otg_board_data;

	jz4770_adc_device.dev.platform_data = &gcw0_battery_pdata;
	jz4770_msc0_device.dev.platform_data = &gcw_internal_sd_data;
	jz4770_msc1_device.dev.platform_data = &gcw_external_sd_data;
	jz4770_icdc_device.dev.platform_data = &gcw0_icdc_pdata;

	return platform_add_devices(jz_platform_devices,
				    ARRAY_SIZE(jz_platform_devices));
}

static unsigned long gpio_charger_pin_cfg[] = {
	PIN_CONFIG_BIAS_DISABLE,
};

static struct pinctrl_map pin_map[] __initdata = {
	PIN_MAP_MUX_GROUP("jz-msc.0", PINCTRL_STATE_DEFAULT,
			  "10010000.jz4770-pinctrl", "msc0_4bit", "msc0"),
	PIN_MAP_MUX_GROUP("jz-msc.1", PINCTRL_STATE_DEFAULT,
			  "10010000.jz4770-pinctrl", "msc1_4bit", "msc1"),
	PIN_MAP_MUX_GROUP("musb-jz.0", PINCTRL_STATE_DEFAULT,
			  "10010000.jz4770-pinctrl", NULL, "otg"),
	PIN_MAP_MUX_GROUP("jz-lcd.0", PINCTRL_STATE_DEFAULT,
			  "10010000.jz4770-pinctrl", "lcd_rgb888", "lcd"),
	PIN_MAP_MUX_GROUP("jz-lcd.0", PINCTRL_STATE_SLEEP,
			  "10010000.jz4770-pinctrl", "no_pins", "lcd"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("gpio-charger.0", "10010000.jz4770-pinctrl",
			  "PF5", gpio_charger_pin_cfg),
	PIN_MAP_CONFIGS_PIN_DEFAULT("gpio-charger.1", "10010000.jz4770-pinctrl",
			  "PB5", gpio_charger_pin_cfg),
};

static void __init board_init_pins(void)
{
	pinctrl_register_mappings(pin_map, ARRAY_SIZE(pin_map));
}

static int __init gcw0_board_setup(void)
{
	printk(KERN_INFO "GCW Zero JZ4770 setup\n");

	board_init_pins();

	if (gcw0_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}

arch_initcall(gcw0_board_setup);

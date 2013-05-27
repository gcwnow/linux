/*
 *  linux/drivers/mmc/host/jz_mmc/gpio/jz_mmc_gpio.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/mmc.h>

#include "include/chip-msc.h"
#include "include/jz_mmc_gpio.h"
#include "include/jz_mmc_host.h"
#include "include/jz_mmc_msc.h"


static irqreturn_t jz_mmc_detect_irq(int irq, void *devid)
{
	struct jz_mmc_host *host = devid;

	mmc_detect_change(host->mmc, HZ / 2);

	return IRQ_HANDLED;
}

static int jz_mmc_request_gpio(struct device *dev, int gpio, const char *name,
			       bool output, int value)
{
	int ret;

	if (!gpio_is_valid(gpio))
		return 0;

	ret = devm_gpio_request(dev, gpio, name);
	if (ret) {
		dev_err(dev, "Failed to request %s gpio: %d\n", name, ret);
		return ret;
	}

	if (output)
		gpio_direction_output(gpio, value);
	else
		gpio_direction_input(gpio);

	return 0;
}

static int jz_mmc_request_card_gpios(struct platform_device *pdev,
				     struct jz_mmc_host *host)
{
	struct device *dev = &pdev->dev;
	struct jz_mmc_platform_data *pdata = dev->platform_data;
	const char *hostname = mmc_hostname(host->mmc);
	int port;
	unsigned int mask;
	unsigned int i;

	/* Determine which pins to use. */
	if (pdata->use_shared_8bit_pins) {
		/* GPE20-29 */
		port = 4;
		mask = BIT(29) | BIT(28) | ((BIT(pdata->bus_width) - 1) << 20);
	} else {
		/* Use private pins for this MSC. */
		if (pdata->bus_width == 8) {
			dev_err(dev, "8-bit bus must use shared pins\n");
			return -EINVAL;
		}
		if (host->pdev_id == 0) {
			/* GPA18-23 */
			port = 0;
			mask = pdata->bus_width == 4 ? 0x00FC0000 : 0x001C0000;
		} else if (host->pdev_id == 1) {
			/* GPD20-25 */
			port = 3;
			mask = pdata->bus_width == 4 ? 0x03F00000 : 0x03100000;
		} else if (host->pdev_id == 2) {
			/* GPB20-21, 28-31 */
			port = 1;
			mask = pdata->bus_width == 4 ? 0xF0300000 : 0x30100000;
		} else {
			BUG();
		}
	}

	/* Request pins. */
	for (i = 0; i < 32; i++) {
		if (mask & BIT(i)) {
			unsigned int pin = port * 32 + i;
			int ret = devm_gpio_request(dev, pin, hostname);
			if (ret) {
				dev_err(dev,
					"Failed to request gpio pin %d: %d\n",
					pin, ret);
				return ret;
			}
		}
	}

	/* Select GPIO function for each pin. */
	if (pdata->use_shared_8bit_pins) {
		REG_GPIO_PXINTC (4) = mask;
		REG_GPIO_PXMASKC(4) = mask;
		if (host->pdev_id == 1)
			REG_GPIO_PXPAT0S(4) = mask;
		else
			REG_GPIO_PXPAT0C(4) = mask;
		if (host->pdev_id == 2)
			REG_GPIO_PXPAT1S(4) = mask;
		else
			REG_GPIO_PXPAT1C(4) = mask;
	} else {
		REG_GPIO_PXINTC (port) = mask;
		REG_GPIO_PXMASKC(port) = mask;
		if (host->pdev_id == 0) {
			REG_GPIO_PXPAT0S(port) = mask & ~BIT(20);
			REG_GPIO_PXPAT0C(port) = mask & BIT(20);
		} else {
			REG_GPIO_PXPAT0C(port) = mask;
		}
		REG_GPIO_PXPAT1C(port) = mask;
	}

	return 0;
}

static int jz_mmc_request_gpios(struct platform_device *pdev,
				struct jz_mmc_host *host)
{
	struct device *dev = &pdev->dev;
	struct jz_mmc_platform_data *pdata = dev->platform_data;
	const char *hostname = mmc_hostname(host->mmc);
	int ret;

	host->label_card_detect = kasprintf(GFP_KERNEL, "%s detect change",
					    hostname);
	ret = jz_mmc_request_gpio(dev, pdata->gpio_card_detect,
				  host->label_card_detect, false, 0);
	if (ret)
		goto err;

	host->label_read_only = kasprintf(GFP_KERNEL, "%s read only", hostname);
	ret = jz_mmc_request_gpio(dev, pdata->gpio_read_only,
				  host->label_read_only, false, 0);
	if (ret)
		goto err;

	host->label_power = kasprintf(GFP_KERNEL, "%s power", hostname);
	ret = jz_mmc_request_gpio(dev, pdata->gpio_power, host->label_power,
				  true, pdata->power_active_low);
	if (ret)
		goto err;

	return 0;

err:
	kfree(host->label_card_detect);
	kfree(host->label_read_only);
	kfree(host->label_power);
	return ret;
}

static int jz_mmc_request_cd_irq(struct platform_device *pdev,
				 struct jz_mmc_host *host)
{
	struct jz_mmc_platform_data *pdata = pdev->dev.platform_data;
	int irq;
	int ret;

	host->card_detect_irq = -1;

	if (!gpio_is_valid(pdata->gpio_card_detect)) {
		dev_info(&pdev->dev, "No card detect facilities available\n");
		return 0;
	}

	irq = gpio_to_irq(pdata->gpio_card_detect);
	if (irq < 0) {
		dev_warn(&pdev->dev,
			 "Failed to look up card detect IRQ: %d\n", irq);
		return 0;
	}

	ret = devm_request_irq(&pdev->dev, irq, jz_mmc_detect_irq,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       host->label_card_detect, host);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "Failed to request card detect IRQ: %d\n", ret);
		return ret;
	}
	host->card_detect_irq = irq;

	return 0;
}

int jz_mmc_gpio_init(struct jz_mmc_host *host, struct platform_device *pdev)
{
	int ret;

	ret = jz_mmc_request_card_gpios(pdev, host);
	if (ret)
		return ret;

	ret = jz_mmc_request_gpios(pdev, host);
	if (ret)
		return ret;

	ret = jz_mmc_request_cd_irq(pdev, host);
	if (ret)
		return ret;

	return 0;
}

void jz_mmc_gpio_deinit(struct jz_mmc_host *host, struct platform_device *pdev)
{
	if (host->card_detect_irq >= 0) {
		disable_irq(host->card_detect_irq);
		device_init_wakeup(&pdev->dev, 0);
	}

	kfree(host->label_card_detect);
	kfree(host->label_read_only);
	kfree(host->label_power);
}

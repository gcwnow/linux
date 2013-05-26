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


#define	TRY_TIME	10
#define	RETRY_TIME	50

#define DETECT_CHANGE_DELAY 50


static void set_card_detect_irq_level(struct jz_mmc_host *host)
{
	if (host->eject ^ host->plat->card_detect_active_low)
		__gpio_as_irq_high_level(host->plat->gpio_card_detect);
	else
		__gpio_as_irq_low_level(host->plat->gpio_card_detect);
}

static void jz_mmc_enable_detect(unsigned long arg)
{
	struct jz_mmc_host *host = (struct jz_mmc_host *)arg;

	atomic_inc(&host->detect_refcnt);

	set_card_detect_irq_level(host);
	enable_irq(host->card_detect_irq);
}

static void jz_mmc_enable_card_detect(struct jz_mmc_host *host)
{
	host->timer.expires = jiffies + DETECT_CHANGE_DELAY * 2;
	host->timer.data = (unsigned long)host;
	host->timer.function = jz_mmc_enable_detect;
	add_timer(&host->timer);
}

static void jiq_de_quiver(struct work_struct *ptr)
{
	struct jz_mmc_host *host = container_of((struct delayed_work *)ptr,
						struct jz_mmc_host, gpio_jiq_work);
	unsigned int time_to_try, i, tmp, counter = 0, result = 1;

	for (time_to_try = 0; time_to_try < RETRY_TIME; time_to_try++) {
		for (i = 0; i < TRY_TIME; i++) {
			tmp = !(gpio_get_value(host->plat->gpio_card_detect) ^
				host->plat->card_detect_active_low);
				// tmp = 1 means slot is empty
			result &= tmp;
			if( !tmp )
				counter++;
			schedule_timeout((10*HZ)/1000);
		}

		if ( !result ) {
			// The card is there
			if (counter == TRY_TIME) {
				host->eject = 0;
				printk("Card Insert\n");
				goto stable;
			}
			/* try again, goto for */
			counter = 0;
			result = 1;
		} else {
			host->eject = 1;
			printk("Card Eject\n");
			goto stable;
		}
	}

stable:
	/* oldstat: 1 -- eject, 0 -- inserted */
	/* eject: 1 -- eject, 0 -- inserted */

#ifdef CONFIG_PM
	if ( (0 == host->oldstat) && (0 == host->eject) && host->sleeping) {
		mmc_resume_host(host->mmc);
	}
#endif

	if ( (0== host->oldstat) && (1 == host->eject) ) {
#ifdef CONFIG_PM
		if (host->sleeping) {
			mmc_resume_host(host->mmc);
		} else
#endif
		{
			mmc_detect_change(host->mmc, 50);
			if (REG_MSC_STAT(host->pdev_id) & MSC_STAT_CLK_EN) {
				printk(" ====> Clock is on\n");
				jz_mmc_reset(host);
			}
		}

		wake_up_interruptible(&host->data_wait_queue);
	}

	if ( (1 == host->oldstat) && (0 == host->eject) ) {
		mmc_detect_change(host->mmc, 50);
	}

	host->sleeping = 0;
	host->oldstat = host->eject;
	jz_mmc_enable_card_detect(host);
}

int jz_mmc_detect(struct jz_mmc_host *host, int from_resuming)
{
	int ret = 0;

	if (!atomic_dec_and_test(&host->detect_refcnt)) {
		atomic_inc(&host->detect_refcnt);
		return 0;
	}

	disable_irq_nosync(host->card_detect_irq);

	if (from_resuming)
		schedule_timeout(HZ / 2); /* 500ms, wait for MMC Block module resuming*/

	ret = schedule_delayed_work( &(host->gpio_jiq_work), HZ / 100); /* 10ms, a little time */

	return ret;
}

#if 0
extern int wait_cmd_done;
extern void jz_mmc_dump_regs(int msc_id, int line);
volatile int error_may_happen = 0;
#endif

static irqreturn_t jz_mmc_detect_irq(int irq, void *devid)
{
#if 0
	printk("===>enter %s\n", __func__);
	if (wait_cmd_done) {
		printk("============================>CAUTION: error may happen!\n");
		//jz_mmc_dump_regs(1, __LINE__);
		error_may_happen = 1;
	}
#endif
	jz_mmc_detect((struct jz_mmc_host *) devid, 0);

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

static int jz_mmc_request_card_gpios(struct jz_mmc_host *host,
				     struct platform_device *pdev)
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

static int jz_mmc_request_gpios(struct jz_mmc_host *host,
				struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jz_mmc_platform_data *pdata = dev->platform_data;
	const char *hostname = mmc_hostname(host->mmc);
	int ret;

	if (!pdata)
		return 0;

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

int jz_mmc_gpio_init(struct jz_mmc_host *host, struct platform_device *pdev)
{
	struct jz_mmc_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	ret = jz_mmc_request_card_gpios(host, pdev);
	if (ret)
		return ret;

	ret = jz_mmc_request_gpios(host, pdev);
	if (ret)
		return ret;

	/*
	 * Setup card detect change
	 */
	host->card_detect_irq = -1;
	if (pdata && gpio_is_valid(pdata->gpio_card_detect)) {
		int irq = gpio_to_irq(pdata->gpio_card_detect);

		device_init_wakeup(&pdev->dev, 1);

		INIT_DELAYED_WORK(&(host->gpio_jiq_work), jiq_de_quiver);
		init_timer(&host->timer);

		atomic_set(&host->detect_refcnt, 1);
		host->sleeping = 0;

		/* Check if there is currently any card present. */
		host->eject = !(gpio_get_value(host->plat->gpio_card_detect) ^
				host->plat->card_detect_active_low);
		host->oldstat = host->eject;
		set_card_detect_irq_level(host);

		ret = devm_request_irq(&pdev->dev,
				       irq,
				       jz_mmc_detect_irq,
				       0,
				       host->label_card_detect,
				       host);
		if (ret < 0) {
			dev_warn(&pdev->dev, "Failed to get card detect IRQ\n");
			return 0;
		}
		host->card_detect_irq = irq;
	} else {
		dev_warn(&pdev->dev, "No card detect facilities available\n");
	}

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

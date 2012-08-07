/*
 *  linux/drivers/mmc/host/jz_mmc/gpio/jz_mmc_gpio.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/mmc/host.h>

#include <asm/mach-jz4770/jz4770msc.h>

#include "include/jz_mmc_gpio.h"
#include "include/jz_mmc_msc.h"


#define	TRY_TIME	10
#define	RETRY_TIME	50

#define DETECT_CHANGE_DELAY 50

static void jz_mmc_enable_detect(unsigned long arg) {
	struct jz_mmc_host *host = (struct jz_mmc_host *)arg;

	atomic_inc(&host->detect_refcnt);

	if (host->eject) {
		/* wait for card insertion */
		host->plat->plug_change(CARD_REMOVED);
	} else {
		/* wait for card removal */
		host->plat->plug_change(CARD_INSERTED);
	}
	enable_irq(host->plat->status_irq);
}

static void jz_mmc_enable_card_detect(struct jz_mmc_host *host) {
	host->timer.expires = jiffies + DETECT_CHANGE_DELAY * 2;
	host->timer.data = (unsigned long)host;
	host->timer.function = jz_mmc_enable_detect;
	add_timer(&host->timer);
}

static void jiq_de_quiver(struct work_struct *ptr){
	struct jz_mmc_host *host = container_of((struct delayed_work *)ptr,
						struct jz_mmc_host, gpio_jiq_work);
	unsigned int time_to_try, i, tmp, counter = 0, result = 1;

	if (unlikely(!host->plat->status)) { /* NEVER */
		mmc_detect_change(host->mmc, 0);
		jz_mmc_enable_card_detect(host);
		return;
	}

	for (time_to_try = 0; time_to_try < RETRY_TIME; time_to_try++) {
		for (i = 0; i < TRY_TIME; i++) {
			tmp = (!host->plat->status(mmc_dev(host->mmc))); // tmp = 1 means slot is empty
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

int jz_mmc_detect(struct jz_mmc_host *host, int from_resuming) {
	int ret = 0;

	if (!atomic_dec_and_test(&host->detect_refcnt)) {
		atomic_inc(&host->detect_refcnt);
		return 0;
	}

	disable_irq_nosync(host->plat->status_irq);

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

static int jz_mmc_gpio_init(struct jz_mmc_host *host, struct platform_device *pdev)
{
	int ret = 0;

	/*
	 * Setup card detect change
	 */
	if (host->plat->status_irq) {
		ret = request_irq(host->plat->status_irq,
				  jz_mmc_detect_irq,
				  0,
				  "jz-msc (gpio)",
				  host);
		if (ret) {
			printk(KERN_ERR "Unable to get slot IRQ %d (%d)\n",
			       host->plat->status_irq, ret);
			return ret;
		}

		device_init_wakeup(&pdev->dev, 1);

		INIT_DELAYED_WORK(&(host->gpio_jiq_work), jiq_de_quiver);
		init_timer(&host->timer);

		atomic_set(&host->detect_refcnt, 1);
		host->sleeping = 0;

		// Check if there were any card present
		if (host->plat->status) {
			host->eject = !(host->plat->status(mmc_dev(host->mmc)));
			host->oldstat = host->eject;

			if(host->eject) {
				host->plat->plug_change(CARD_REMOVED);
			} else {
				host->plat->plug_change(CARD_INSERTED);
			}
		}
	} else
		printk(KERN_ERR "%s: No card detect facilities available\n",
		       mmc_hostname(host->mmc));

	return 0;
}

static void jz_mmc_gpio_deinit(struct jz_mmc_host *host, struct platform_device *pdev)
{
	if(host->plat->status_irq) {
		free_irq(host->plat->status_irq, host);
		device_init_wakeup(&pdev->dev, 0);
	}
}

int jz_mmc_gpio_register(struct jz_mmc_gpio *gpio)
{
	if(gpio == NULL)
		return -ENOMEM;

	gpio->init = jz_mmc_gpio_init;
	gpio->deinit = jz_mmc_gpio_deinit;

	return 0;
}

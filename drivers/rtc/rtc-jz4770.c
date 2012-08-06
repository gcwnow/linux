/*
 * Real Time Clock interface for Jz4770.
 *
 * Copyright (C) 2005-2009, Ingenic Semiconductor Inc.
 *
 * Author: Richard Feng <cjfeng@ingenic.cn>
 *         Regen Huang <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/bitops.h>

#include <asm/irq.h>
#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770rtc.h>


#define TIMER_FREQ		CLOCK_TICK_RATE

/* The divider is decided by the RTC clock frequency. */
#define RTC_FREQ_DIVIDER	(32768 - 1)

#define ms2clycle(x)  (((x) * RTC_FREQ_DIVIDER) / 1000)

/* Default time for the first-time power on */
static struct rtc_time default_tm = {
	.tm_year = (2010 - 1900), // year 2010
	.tm_mon = (10 - 1),       // month 10
	.tm_mday = 1,             // day 1
	.tm_hour = 12,
	.tm_min = 0,
	.tm_sec = 0
};

static unsigned long rtc_freq = 1024;
static struct rtc_time rtc_alarm;
static DEFINE_SPINLOCK(jz4770_rtc_lock);

static inline int rtc_periodic_alarm(struct rtc_time *tm)
{
	return  (tm->tm_year == -1) ||
		((unsigned)tm->tm_mon >= 12) ||
		((unsigned)(tm->tm_mday - 1) >= 31) ||
		((unsigned)tm->tm_hour > 23) ||
		((unsigned)tm->tm_min > 59) ||
		((unsigned)tm->tm_sec > 59);
}

/*
 * Calculate the next alarm time given the requested alarm time mask
 * and the current time.
 */
static void rtc_next_alarm_time(struct rtc_time *next, struct rtc_time *now,
				struct rtc_time *alrm)
{
	unsigned long next_time;
	unsigned long now_time;

	next->tm_year = now->tm_year;
	next->tm_mon = now->tm_mon;
	next->tm_mday = now->tm_mday;
	next->tm_hour = alrm->tm_hour;
	next->tm_min = alrm->tm_min;
	next->tm_sec = alrm->tm_sec;

	rtc_tm_to_time(now, &now_time);
	rtc_tm_to_time(next, &next_time);

	if (next_time < now_time) {
		/* Advance one day */
		next_time += 60 * 60 * 24;
		rtc_time_to_tm(next_time, next);
	}
}
static int rtc_update_alarm(unsigned int now, struct rtc_time *alrm,
			    unsigned long *time)
{
	struct rtc_time alarm_tm, now_tm;
	rtc_time_to_tm(now, &now_tm);
	rtc_next_alarm_time(&alarm_tm, &now_tm, alrm); //?
	return rtc_tm_to_time(&alarm_tm, time);
}

#define IS_RTC_IRQ(x,y)  (((x) & (y)) == (y))

static irqreturn_t jz4770_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned int rtsr,save_rtsr;
	unsigned long events;

	spin_lock(&jz4770_rtc_lock);
	rtsr = rtc_read_reg(RTC_RTCCR);
	save_rtsr = rtsr;
	//is rtc interrupt
	events = 0;
	if(IS_RTC_IRQ(rtsr,RTC_RTCCR_AF))
	{
	  events = RTC_AF | RTC_IRQF;
	  rtsr &= ~RTC_RTCCR_AF;
	  if(rtc_periodic_alarm(&rtc_alarm) == 0)
	    rtsr &= ~(RTC_RTCCR_AIE | RTC_RTCCR_AE);
	  else
	  {
	    unsigned int now = rtc_read_reg(RTC_RTCSR);
	    unsigned long time;
	    rtc_update_alarm(now,&rtc_alarm,&time);
	    rtc_write_reg(RTC_RTCSAR,time);

	  }
	}
	if(IS_RTC_IRQ(rtsr,RTC_RTCCR_1HZ))
	{
	  rtsr &= ~(RTC_RTCCR_1HZ);
	  events = RTC_UF | RTC_IRQF;
	}
	if(events != 0)
	  rtc_update_irq(rtc, 1, events);
	if(rtsr != save_rtsr)
	  rtc_write_reg(RTC_RTCCR,rtsr);

	spin_unlock(&jz4770_rtc_lock);

	return IRQ_HANDLED;
}

#if 0
static int rtc_timer1_count;
static irqreturn_t timer1_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	/*
	 * If we match for the first time, rtc_timer1_count will be 1.
	 * Otherwise, we wrapped around (very unlikely but
	 * still possible) so compute the amount of missed periods.
	 * The match reg is updated only when the data is actually retrieved
	 * to avoid unnecessary interrupts.
	 */
	OSSR = OSSR_M1;	/* clear match on timer1 */

	rtc_update_irq(rtc, rtc_timer1_count, RTC_PF | RTC_IRQF);

	if (rtc_timer1_count == 1)
		rtc_timer1_count = (rtc_freq * ((1<<30)/(TIMER_FREQ>>2)));

	return IRQ_HANDLED;
}
#endif

#if 0
static int jz4770_rtc_read_callback(struct device *dev, int data)
{
	if (data & RTC_PF) {
		/* interpolate missed periods and set match for the next */
		unsigned long period = TIMER_FREQ/rtc_freq;
		unsigned long oscr = OSCR;
		unsigned long osmr1 = OSMR1;
		unsigned long missed = (oscr - osmr1)/period;
		data += missed << 8;
		OSSR = OSSR_M1;	/* clear match on timer 1 */
		OSMR1 = osmr1 + (missed + 1)*period;
		/* Ensure we didn't miss another match in the mean time.
		 * Here we compare (match - OSCR) 8 instead of 0 --
		 * see comment in pxa_timer_interrupt() for explanation.
		 */
		while( (signed long)((osmr1 = OSMR1) - OSCR) <= 8 ) {
			data += 0x100;
			OSSR = OSSR_M1;	/* clear match on timer 1 */
			OSMR1 = osmr1 + period;
		}
	}
	return data;
}
#endif

static int jz4770_rtc_open(struct device *dev)
{
	int ret;

	ret = request_irq(IRQ_RTC, jz4770_rtc_interrupt, IRQF_DISABLED,
				"rtc 1Hz and alarm", dev);
	if (ret) {
		dev_err(dev, "IRQ %d already in use.\n", IRQ_RTC);
		goto fail_ui;
	}

	/*ret = request_irq(IRQ_OST1, timer1_interrupt, IRQF_DISABLED,
				"rtc timer", dev);
	if (ret) {
		dev_err(dev, "IRQ %d already in use.\n", IRQ_OST1);
		goto fail_pi;
		}*/

	return 0;

fail_ui:
	free_irq(IRQ_RTC, dev);
	return ret;
}

static void jz4770_rtc_release(struct device *dev)
{
	spin_lock_irq(&jz4770_rtc_lock);

	spin_unlock_irq(&jz4770_rtc_lock);
	//free_irq(IRQ_OST1, dev);
	free_irq(IRQ_RTC, dev);
}

static int jz4770_rtc_ioctl(struct device *dev, unsigned int cmd,
			    unsigned long arg)
{
	unsigned int tmp;
	switch (cmd) {
	case RTC_AIE_OFF:
	        spin_lock_irq(&jz4770_rtc_lock);
		rtc_clr_reg(RTC_RTCCR,
			    RTC_RTCCR_AIE | RTC_RTCCR_AE | RTC_RTCCR_AF);
		spin_unlock_irq(&jz4770_rtc_lock);
		return 0;
	case RTC_AIE_ON:
		spin_lock_irq(&jz4770_rtc_lock);
		tmp = rtc_read_reg(RTC_RTCCR);
		tmp &= ~RTC_RTCCR_AF;
		tmp |= RTC_RTCCR_AIE | RTC_RTCCR_AE;
		rtc_write_reg(RTC_RTCCR, tmp);
		spin_unlock_irq(&jz4770_rtc_lock);
		return 0;
	case RTC_UIE_OFF:
		spin_lock_irq(&jz4770_rtc_lock);
		rtc_clr_reg(RTC_RTCCR, RTC_RTCCR_1HZ | RTC_RTCCR_1HZIE);
		spin_unlock_irq(&jz4770_rtc_lock);
		return 0;
	case RTC_UIE_ON:
		spin_lock_irq(&jz4770_rtc_lock);
		tmp = rtc_read_reg(RTC_RTCCR);
		tmp &= ~RTC_RTCCR_1HZ;
		tmp |= RTC_RTCCR_1HZIE;
		rtc_write_reg(RTC_RTCCR, tmp);
		spin_unlock_irq(&jz4770_rtc_lock);
		return 0;
	case RTC_PIE_OFF:
		spin_lock_irq(&jz4770_rtc_lock);
		printk("no implement!\n");
		spin_unlock_irq(&jz4770_rtc_lock);
		return 0;
	case RTC_PIE_ON:
		spin_lock_irq(&jz4770_rtc_lock);
		printk("no implement!\n");
		spin_unlock_irq(&jz4770_rtc_lock);
		return 0;
	case RTC_IRQP_READ:
		return put_user(rtc_freq, (unsigned long *)arg);
	case RTC_IRQP_SET:
		if (arg < 1 || arg > TIMER_FREQ)
			return -EINVAL;
		rtc_freq = arg;
		return 0;
	}
	return -ENOIOCTLCMD;
}

static int jz4770_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	int ret;

	ret = rtc_tm_to_time(tm, &time);
	if (ret == 0) {
	  rtc_write_reg(RTC_RTCSR, time);
	}
	return ret;
}

static int jz4770_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned int tmp;
	tmp = rtc_read_reg(RTC_RTCSR);
	rtc_time_to_tm(tmp, tm);

	if (rtc_valid_tm(tm) < 0) {
		/* Set the default time */
		jz4770_rtc_set_time(dev, &default_tm);
		tmp = rtc_read_reg(RTC_RTCSR);
		rtc_time_to_tm(tmp, tm);
	}
	return 0;
}

static int jz4770_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned int rtc_rcr,tmp;
	tmp = rtc_read_reg(RTC_RTCSAR);
	rtc_time_to_tm(tmp, &rtc_alarm);
	memcpy(&alrm->time, &rtc_alarm, sizeof(struct rtc_time));
	rtc_rcr = rtc_read_reg(RTC_RTCCR);
	alrm->enabled = (rtc_rcr & RTC_RTCCR_AIE) ? 1 : 0;
	alrm->pending = (rtc_rcr & RTC_RTCCR_AF) ? 1 : 0;
	return 0;
}

static int jz4770_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	int ret = 0;
	unsigned int now;
	unsigned long time;
	unsigned int tmp;
	spin_lock_irq(&jz4770_rtc_lock);
	now = rtc_read_reg(RTC_RTCSR);
	rtc_update_alarm(now,&alrm->time,&time);
	rtc_write_reg(RTC_RTCSAR, time);
	if(alrm->enabled){
	  tmp = rtc_read_reg(RTC_RTCCR);
	  tmp &= ~RTC_RTCCR_AF;
	  tmp |= RTC_RTCCR_AIE | RTC_RTCCR_AE;
	  rtc_write_reg(RTC_RTCCR, tmp);
	}else{
	  rtc_clr_reg(RTC_RTCCR, RTC_RTCCR_AIE | RTC_RTCCR_AE | RTC_RTCCR_AF);
	}
	spin_unlock_irq(&jz4770_rtc_lock);

	return ret;
}

static int jz4770_rtc_proc(struct device *dev, struct seq_file *seq)
{
	seq_printf(seq, "RTC regulator\t: 0x%08x\n", rtc_read_reg(RTC_RTCGR));
	seq_printf(seq, "update_IRQ\t: %s\n",
			(rtc_read_reg(RTC_RTCCR) & RTC_RTCCR_1HZIE)
				? "yes" : "no");
	/*seq_printf(seq, "periodic_IRQ\t: %s\n",
	  (OIER & OIER_E1) ? "yes" : "no");*/
	seq_printf(seq, "periodic_freq\t: %ld\n", rtc_freq);

	return 0;
}

static const struct rtc_class_ops jz4770_rtc_ops = {
	.open = jz4770_rtc_open,
	//.read_callback = jz4770_rtc_read_callback,
	.release = jz4770_rtc_release,
	.ioctl = jz4770_rtc_ioctl,
	.read_time = jz4770_rtc_read_time,
	.set_time = jz4770_rtc_set_time,
	.read_alarm = jz4770_rtc_read_alarm,
	.set_alarm = jz4770_rtc_set_alarm,
	.proc = jz4770_rtc_proc,
};

static int __devinit jz4770_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	unsigned int cfc,hspr,rgr_1hz;

	/*
	 * When we are powered on for the first time, init the rtc and reset time.
	 *
	 * For other situations, we remain the rtc status unchanged.
	 */
	cpm_set_clock(CGU_RTCCLK, 32768);

	//unsigned int ppr = IN_RTC_REG(REG_RTC_HWRSR);
	cfc = HSPR_RTCV;
	hspr = rtc_read_reg(RTC_HSPR);
	rgr_1hz = rtc_read_reg(RTC_RTCGR) & RTC_RTCGR_NC1HZ_MASK;

	if((hspr != cfc) || (rgr_1hz != RTC_FREQ_DIVIDER)) {
		/* We are powered on for the first time !!! */
		printk("jz4770-rtc: rtc status reset by power-on\n");
		/* Set 32768 rtc clocks per seconds */
		rtc_write_reg(RTC_RTCGR, RTC_FREQ_DIVIDER);
		/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
		rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(100));
		rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(60));

                /* Reset to the default time */
		jz4770_rtc_set_time(NULL, &default_tm);

		/* start rtc */
		//rtc_write_reg(RTC_RTCCR,
		//	      RTC_RTCCR_RTCE | RTC_RTCCR_AE | RTC_RTCCR_AIE);
		rtc_write_reg(RTC_RTCCR, RTC_RTCCR_RTCE)
		rtc_write_reg(RTC_HSPR, cfc);
	}

	/* clear all rtc flags */
	rtc_write_reg(RTC_HWRSR, 0);

	device_init_wakeup(&pdev->dev, 1);
	rtc = rtc_device_register(pdev->name, &pdev->dev, &jz4770_rtc_ops,
				THIS_MODULE);

	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(pdev, rtc);

	rtc_write_reg(RTC_CKPCR, 0x0);

	return 0;
}

static int __devexit jz4770_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	rtc_write_reg(RTC_RTCCR, 0);
	if (rtc)
		rtc_device_unregister(rtc);
	return 0;
}

#ifdef CONFIG_PM
static int jz4770_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	//if (device_may_wakeup(&pdev->dev))
	//	enable_irq_wake(IRQ_RTC);
	return 0;
}

static int jz4770_rtc_resume(struct platform_device *pdev)
{
	//if (device_may_wakeup(&pdev->dev))
	//	disable_irq_wake(IRQ_RTC);
	return 0;
}
#else
#define jz4770_rtc_suspend	NULL
#define jz4770_rtc_resume	NULL
#endif

static struct platform_driver jz4770_rtc_driver = {
	.probe			= jz4770_rtc_probe,
	.remove			= __devexit_p(jz4770_rtc_remove),
	.suspend		= jz4770_rtc_suspend,
	.resume			= jz4770_rtc_resume,
	.driver = {
		.name		= "jz4770-rtc",
	},
};

module_platform_driver(jz4770_rtc_driver);

MODULE_AUTHOR("James Jia <ljia@ingenic.cn>");
MODULE_DESCRIPTION("JZ4770 Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz4770-rtc");

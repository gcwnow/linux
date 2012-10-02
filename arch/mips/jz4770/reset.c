/*
 * linux/arch/mips/jz4770/reset.c
 *
 * JZ4770 reset routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <yliu@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <asm/reboot.h>

#include "intc.h"
#include "rtc.h"

void jz_halt(void)
{
	printk(KERN_NOTICE "\n** You can safely turn off the power\n");

	while (1)
		__asm__(".set\tmips3\n\t"
	                "wait\n\t"
			".set\tmips0");
}

void jz_power_off(void)
{
	printk("Put CPU into hibernate mode.\n");

	/* Mask all interrupts */
	REG32(INTC_ICMSR(0)) = 0xffffffff;
	REG32(INTC_ICMSR(1)) = 0x7ff;

	/*
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(100));

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(60));

	/* clear wakeup status register */
	rtc_write_reg(RTC_HWRSR, 0x0);

	/* set wake up valid level as low  and disable rtc alarm wake up.*/
        rtc_write_reg(RTC_HWCR,0x8);

	/* Put CPU to hibernate mode */
	rtc_write_reg(RTC_HCR, RTC_HCR_PD);

	while (1) {
		printk("We should NOT come here, please check the jz4770rtc.h!!!\n");
	};
}

void jz_reset_init(void)
{
	_machine_halt = jz_halt;
	pm_power_off = jz_power_off;
}

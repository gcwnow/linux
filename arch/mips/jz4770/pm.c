/*
 * linux/arch/mips/jz4770/pm.c
 *
 * JZ4770 Power Management Routines
 *
 * Copyright (C) 2006 - 2010 Ingenic Semiconductor Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/delay.h>

#include <asm/cacheops.h>

#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770misc.h>
#include <asm/mach-jz4770/jz4770msc.h>
#include <asm/mach-jz4770/jz4770rtc.h>
#include <asm/mach-jz4770/jz4770sadc.h>


#define CONFIG_PM_POWERDOWN_P0 y
#define JZ_PM_SIMULATE_BATTERY y

#ifdef JZ_PM_SIMULATE_BATTERY
#define CONFIG_BATTERY_JZ
#define JZ_PM_BATTERY_SIMED
#endif

#if defined(CONFIG_RTC_DRV_JZ4770) && defined(CONFIG_BATTERY_JZ)
//extern unsigned long jz_read_bat(void);
//extern int g_jz_battery_min_voltage;
static unsigned int usr_alarm_data = 0;
static int alarm_state = 0;
#endif

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

extern void jz_board_do_sleep(unsigned long *ptr);
extern void jz_board_do_resume(unsigned long *ptr);
#if defined(CONFIG_PM_POWERDOWN_P0)
extern void jz_cpu_sleep(void);
extern void jz_cpu_resume(void);
#endif

#if defined(CONFIG_RTC_DRV_JZ4770) && defined(CONFIG_BATTERY_JZ)
static int alarm_remain = 0;
//#define ALARM_TIME (1 * 60)
#define ALARM_TIME (10 * 60)
static inline void jz_save_alarm(void) {
	uint32_t rtc_rtcsr = 0,rtc_rtcsar = 0;

	rtc_rtcsar = rtc_read_reg(RTC_RTCSAR); /* second alarm register */
	rtc_rtcsr = rtc_read_reg(RTC_RTCSR); /* second register */

	alarm_remain = rtc_rtcsar - rtc_rtcsr;
}

static inline void jz_restore_alarm(void) {
	if (alarm_remain > 0) {
		rtc_write_reg(RTC_RTCSAR, rtc_read_reg(RTC_RTCSR) + alarm_remain);
		rtc_set_reg(RTC_RTCCR,0x3<<2); /* alarm enable, alarm interrupt enable */
	}
}

static void jz_set_alarm(void)
{
	uint32_t rtc_rtcsr = 0,rtc_rtcsar = 0;

	rtc_rtcsar = rtc_read_reg(RTC_RTCSAR); /* second alarm register */
	rtc_rtcsr = rtc_read_reg(RTC_RTCSR); /* second register */
#if 0
	if(rtc_rtcsar <= rtc_rtcsr) {
#endif
		printk("1\n");
		rtc_write_reg(RTC_RTCSAR,rtc_rtcsr + ALARM_TIME);
		rtc_set_reg(RTC_RTCCR,0x3<<2); /* alarm enable, alarm interrupt enable */
		alarm_state = 1;	       /* alarm on */
#if 0
	} else if(rtc_rtcsar > rtc_rtcsr + ALARM_TIME) {
		printk("2\n");
		usr_alarm_data = rtc_rtcsar;
		rtc_write_reg(RTC_RTCSAR,rtc_rtcsr + ALARM_TIME);
		rtc_set_reg(RTC_RTCCR,0x3<<2);
		alarm_state = 1;
	} else {	      /* ??? I have some questions here, when the cpu is sleeping, the time freezes, doesn't it?
				 consider sleep->wakeup->sleep   --- by Lutts */
		printk("3\n");
		usr_alarm_data = 0;
		rtc_set_reg(RTC_RTCCR,0x3<<2);
		alarm_state = 0;
	}
#endif

	rtc_rtcsar = rtc_read_reg(RTC_RTCSAR); /* second alarm register */
	rtc_rtcsr = rtc_read_reg(RTC_RTCSR); /* second register */

	printk("%s: rtc_rtcsar = %u rtc_rtcsr = %u alarm_state = %d\n", __func__, rtc_rtcsar, rtc_rtcsr, alarm_state);
}
#undef ALARM_TIME
#endif

#define DIV_1 0
#define DIV_2 1
#define DIV_3 2
#define DIV_4 3
#define DIV_6 4
#define DIV_8 5
#define DIV_12 6
#define DIV_16 7
#define DIV_24 8
#define DIV_32 9

#define DIV(a,b,c,d,e,f)					\
	({								\
	 unsigned int retval;					\
	 retval = (DIV_##a << CPCCR_CDIV_LSB)   |		\
	 (DIV_##b << CPCCR_H0DIV_LSB)  |		\
	 (DIV_##c << CPCCR_PDIV_LSB)   |		\
	 (DIV_##d << CPCCR_C1DIV_LSB)  |		\
	 (DIV_##e << CPCCR_H2DIV_LSB)  |		\
	 (DIV_##f << CPCCR_H1DIV_LSB);			\
	 retval;							\
	 })

#define cache_prefetch(label)						\
do{									\
	unsigned long addr,size,end;					\
	/* Prefetch codes from label */					\
	addr = (unsigned long)(&&label) & ~(32 - 1);			\
	size = 32 * 256; /* load 128 cachelines */			\
	end = addr + size;						\
	for (; addr < end; addr += 32) {				\
		__asm__ volatile (					\
				".set mips32\n\t"			\
				" cache %0, 0(%1)\n\t"			\
				".set mips32\n\t"			\
				:					\
				: "I" (Index_Prefetch_I), "r"(addr));	\
	}								\
}									\
while(0)

unsigned int jz_set_div(unsigned int div)
{
	unsigned int cpccr;
	unsigned int retval;

	cpccr = REG_CPM_CPCCR;
	retval = cpccr;
	cpccr = (cpccr & CPCCR_MEM) | CPCCR_CE | div;
	cache_prefetch(jz_set_div_L1);
jz_set_div_L1:
	REG_CPM_CPPSR = 0x1;
	REG_CPM_CPCCR = cpccr;
	while (!(REG_CPM_CPPCR0 & CPPCR0_PLLS));

	return retval;
}

static int jz_pm_do_sleep(void)
{
	unsigned int div;
	//unsigned long delta;
	//unsigned long nfcsr = REG_NEMC_NFCSR;
	unsigned long opcr = INREG32(CPM_OPCR);
	unsigned long icmr0 = INREG32(INTC_ICMR(0));
	unsigned long icmr1 = INREG32(INTC_ICMR(1));
	unsigned long sadc = REG_SADC_ENA;
	unsigned long sleep_gpio_save[7*(GPIO_PORT_NUM-1)];
	unsigned long cpuflags;
#ifdef CONFIG_JZ_SYSTEM_AT_CARD
	unsigned int lpm;
#endif

#if defined(CONFIG_RTC_DRV_JZ4770) && defined(CONFIG_BATTERY_JZ)
	jz_save_alarm();

 __jz_pm_do_sleep_start:
#endif
	printk("Put CPU into sleep mode.\n");

	/* set SLEEP mode */
	CMSREG32(CPM_LCR, LCR_LPM_SLEEP, LCR_LPM_MASK);

	/* Preserve current time */
	// TODO(MtH): The generic kernel code should take care of this,
	//            I think - verify that.
	//            The JZ4740 code doesn't do anything special with time.
	//            In any case, we can't write the xtime struct directly
	//            anymore and there is no simple accessor function for it
	//            either, so a new approach is needed.
	//delta = xtime.tv_sec - REG_RTC_RTCSR;

	/* Save CPU irqs */
	local_irq_save(cpuflags);

        /* Disable nand flash */
	//REG_NEMC_NFCSR = ~0xff;

        /* stop sadc */
	REG_SADC_ENA |= SADC_ENA_POWER;
	while ((REG_SADC_ENA & SADC_ENA_POWER) != SADC_ENA_POWER) {
		dprintk("REG_SADC_ENA = 0x%x\n", REG_SADC_ENA);
		udelay(100);
	}

        /* stop uhc */
	SETREG32(CPM_OPCR, OPCR_UHCPHY_DISABLE);

	/* stop otg and gps */
	CLRREG32(CPM_OPCR, OPCR_OTGPHY_ENABLE | OPCR_GPSEN);

	/*power down gps and ahb1*/
	//SETREG32(CPM_LCR, LCR_PDAHB1 | LCR_PDGPS);

	//while(!(REG_CPM_LCR && LCR_PDAHB1S)) ;
	//while(!(REG_CPM_LCR && LCR_PDGPSS)) ;

	/* Mask all interrupts */

	OUTREG32(INTC_ICMSR(0), 0xffffffff);
	OUTREG32(INTC_ICMSR(1), 0xbfff);

#if defined(CONFIG_RTC_DRV_JZ4770)
	OUTREG32(INTC_ICMCR(1), 0x1);
	jz_set_alarm();
	__intc_ack_irq(IRQ_RTC);
	__intc_unmask_irq(IRQ_RTC); //unmask rtc irq
	rtc_clr_reg(RTC_RTCCR,RTC_RTCCR_AF);
#else
	/* mask rtc interrupts */
	OUTREG32(INTC_ICMSR(1), 0x1);
#endif

	/* Sleep on-board modules */
	jz_board_do_sleep(sleep_gpio_save);

	printk("control = 0x%08x icmr0 = 0x%08x icmr1 = 0x%08x\n",
	       INREG32(RTC_RTCCR), INREG32(INTC_ICMR(0)), INREG32(INTC_ICMR(1)));

	/* disable externel clock Oscillator in sleep mode */
	CLRREG32(CPM_OPCR, OPCR_O1SE);

	/* select 32K crystal as RTC clock in sleep mode */
	SETREG32(CPM_OPCR, OPCR_ERCS);

	/* like jz4760b, for cpu 533M 1:2:4 */
	OUTREG32(CPM_PSWC0ST, 0);
	OUTREG32(CPM_PSWC1ST, 8);
	OUTREG32(CPM_PSWC2ST, 11);
	OUTREG32(CPM_PSWC3ST, 0);

#if defined(CONFIG_PM_POWERDOWN_P0)
	printk("Shutdown P0\n");

	/* power down the p0 */
	SETREG32(CPM_OPCR, OPCR_PD);

	/* Clear previous reset status */
	CLRREG32(CPM_RSR, RSR_PR | RSR_WR | RSR_P0R);

	/* Set resume return address */
	OUTREG32(CPM_CPSPPR, 0x00005a5a);
	udelay(1);
	OUTREG32(CPM_CPSPR, virt_to_phys(jz_cpu_resume));

	rtc_clr_reg(RTC_RTCCR,RTC_RTCCR_AF);

#ifdef CONFIG_JZ_SYSTEM_AT_CARD
	lpm = REG_MSC_LPM(0);
#endif
	/* *** go zzz *** */

	div = DIV(2,4,8,2,4,4);
	div = jz_set_div(div);

	jz_cpu_sleep();

	div = DIV(1,4,8,2,4,4);
	jz_set_div(div);

#ifdef CONFIG_JZ_SYSTEM_AT_CARD
	REG_MSC_LPM(0) = lpm;
#endif

#else
	{
		unsigned long addr, size, end, cnt;

		__asm__(".set\tmips32\n\t"
			"sync\n\t"
			".set\tmips32");

		/* Prefetch codes from L1 */
		addr = (unsigned long)(&&L1) & ~(32 - 1);
		size = 32 * 128; /* load 128 cachelines */
		end = addr + size;

		for (; addr < end; addr += 32) {
			__asm__ volatile (
					".set mips32\n\t"
					" cache %0, 0(%1)\n\t"
					".set mips32\n\t"
					:
					: "I" (Index_Prefetch_I), "r"(addr));
		}

		/* wait for EMC stable */
		cnt = 0x3ffff;
		while(cnt--);


		/* Start of the prefetching codes */
L1:
		*((volatile unsigned int *)0xb3020050) = 0xff00ff00;


		__asm__ volatile (".set\tmips32\n\t"
				"wait\n\t"
				"nop\n\t"
				"nop\n\t"
				"nop\n\t"
				"nop\n\t"
				".set\tmips32");

		*((volatile unsigned int *)0xb3020050) = 0x0000ff00;
L2:

		/* End of the prefetching codes */
	}
#endif

	/*if power down p0 ,return from sleep.S*/

	/* Restore to IDLE mode */
	CMSREG32(CPM_LCR, LCR_LPM_IDLE, LCR_LPM_MASK);

	/* Restore nand flash control register, it must be restored,
	   because it will be clear to 0 in bootrom. */
	//REG_NEMC_NFCSR = nfcsr;


	/* Restore interrupts FIXME:*/
	OUTREG32(INTC_ICMR(0), icmr0);
	OUTREG32(INTC_ICMR(1), icmr1);

	/* Restore sadc */
	REG_SADC_ENA = sadc;

	/* Resume on-board modules */
	jz_board_do_resume(sleep_gpio_save);

	/* Restore Oscillator and Power Control Register */
	OUTREG32(CPM_OPCR, opcr);

	/* Restore CPU interrupt flags */
	local_irq_restore(cpuflags);

	/* Restore current time */
	//xtime.tv_sec = REG_RTC_RTCSR + delta;

	printk("Resume CPU from sleep mode.\n");

	CLRREG32(CPM_RSR, RSR_PR | RSR_WR | RSR_P0R);

	printk("===>Leave CPU Sleep\n");
#if  defined(CONFIG_RTC_DRV_JZ4770) && defined(CONFIG_BATTERY_JZ)
	if((INREG32(RTC_RTCCR) & RTC_RTCCR_AF)) {
		rtc_clr_reg(RTC_RTCCR, RTC_RTCCR_AF | RTC_RTCCR_AE | RTC_RTCCR_AIE);
		if(!usr_alarm_data) /* restore usrs alarm state */
			rtc_write_reg(RTC_RTCSAR,usr_alarm_data);
#if 0
		if(g_jz_battery_min_voltage > jz_read_bat()) /* Just for example, add your Battery check here */
			pm_power_off();
		else
#endif
			goto __jz_pm_do_sleep_start;
	}
#endif
#if defined(CONFIG_RTC_DRV_JZ4770) && defined(CONFIG_BATTERY_JZ)
	jz_restore_alarm();
#endif
	return 0;
}

#define K0BASE  KSEG0
void jz_flush_cache_all(void)
{
	unsigned long addr;

	/* Clear CP0 TagLo */
	asm volatile ("mtc0 $0, $28\n\t"::);

	for (addr = K0BASE; addr < (K0BASE + 0x4000); addr += 32) {
		asm volatile (
			".set mips3\n\t"
			" cache %0, 0(%1)\n\t"
			".set mips2\n\t"
			:
			: "I" (Index_Writeback_Inv_D), "r"(addr));

		asm volatile (
			".set mips3\n\t"
			" cache %0, 0(%1)\n\t"
			".set mips2\n\t"
			:
			: "I" (Index_Store_Tag_I), "r"(addr));
	}

	asm volatile ("sync\n\t"::);

	/* invalidate BTB */
	asm volatile (
		".set mips32\n\t"
		" mfc0 %0, $16, 7\n\t"
		" nop\n\t"
		" ori $0, 2\n\t"
		" mtc0 %0, $16, 7\n\t"
		" nop\n\t"
		".set mips2\n\t"
		:
		: "r"(addr));
}

static int jz4770_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

/*
 * Jz CPU enter save power mode
 */
static int jz4770_pm_enter(suspend_state_t state)
{
	jz_pm_do_sleep();
	return 0;
}

static struct platform_suspend_ops jz4770_pm_ops = {
	.valid		= jz4770_pm_valid,
	.enter		= jz4770_pm_enter,
};

/*
 * Initialize power interface
 */
int __init jz_pm_init(void)
{
	printk("Power Management for JZ\n");

	suspend_set_ops(&jz4770_pm_ops);
	return 0;
}

#ifdef JZ_PM_BATTERY_SIMED
#undef CONFIG_BATTERY_JZ
#endif

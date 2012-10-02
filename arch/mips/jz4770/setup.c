/*
 * linux/arch/mips/jz4770/common/setup.c
 *
 * JZ4770 common setup routines.
 *
 * Copyright (C) 2011 Ingenic Semiconductor Inc.
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
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>
#include <asm/pgtable.h>
#include <asm/time.h>

#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770uart.h>

#ifdef CONFIG_PM
#include <asm/suspend.h>
#endif

#ifdef CONFIG_PC_KEYB
#include <asm/keyboard.h>
#endif

#include "reset.h"


jz_clocks_t jz_clocks;

extern void __init jz_board_setup(void);

static void __init sysclocks_setup(void)
{
	jz_clocks.cclk = cpm_get_clock(CGU_CCLK);
	jz_clocks.hclk = cpm_get_clock(CGU_HCLK);
	jz_clocks.pclk = cpm_get_clock(CGU_PCLK);
	jz_clocks.c1clk = cpm_get_clock(CGU_C1CLK);
	jz_clocks.mclk = cpm_get_clock(CGU_MCLK);
	jz_clocks.h1clk = cpm_get_clock(CGU_H1CLK);
	jz_clocks.pixclk = cpm_get_clock(CGU_LPCLK);
	jz_clocks.i2sclk = cpm_get_clock(CGU_I2SCLK);
	jz_clocks.otgclk = cpm_get_clock(CGU_OTGCLK);
	jz_clocks.msc0clk = cpm_get_clock(CGU_MSC0CLK);
	jz_clocks.msc1clk = cpm_get_clock(CGU_MSC1CLK);
	jz_clocks.msc2clk = cpm_get_clock(CGU_MSC2CLK);
	jz_clocks.extalclk = __cpm_get_extalclk();
	jz_clocks.rtcclk = __cpm_get_rtcclk();

	printk("CPU clock: %dMHz, System clock: %dMHz, Peripheral clock: %dMHz, Memory clock: %dMHz\n",
	       (jz_clocks.cclk + 500000) / 1000000,
	       (jz_clocks.hclk + 500000) / 1000000,
	       (jz_clocks.pclk + 500000) / 1000000,
	       (jz_clocks.mclk + 500000) / 1000000);
}

static void __init soc_cpm_setup(void)
{
	/* Enable device DMA */
	cpm_start_clock(CGM_DMAC);

	/* CPU enters IDLE mode when executing 'wait' instruction */
	CMSREG32(CPM_LCR, LCR_LPM_IDLE, LCR_LPM_MASK);

	/* Setup system clocks */
	sysclocks_setup();
}

static void __init soc_harb_setup(void)
{
//	__harb_set_priority(0x00);  /* CIM>LCD>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x03);  /* LCD>CIM>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x0a);  /* ETH>LCD>CIM>DMA>PCI>USB>CBB */
}

static void __init soc_dmac_setup(void)
{
	__dmac_enable_module(0);
	__dmac_enable_module(1);
}

static void __init jz_soc_setup(void)
{
	soc_cpm_setup();
	soc_harb_setup();
	soc_dmac_setup();
}

static void __init jz_serial_setup(void)
{
#ifdef CONFIG_SERIAL_8250
	struct uart_port s;

	REG8(UART0_FCR) |= UARTFCR_UUE; /* enable UART module */
	memset(&s, 0, sizeof(s));
	s.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	s.iotype = SERIAL_IO_MEM;
	s.regshift = 2;
	s.uartclk = jz_clocks.extalclk ;

	s.line = 0;
	s.membase = (u8 *)UART0_BASE;
	s.irq = IRQ_UART0;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS0 setup failed!\n");
	}

	s.line = 1;
	s.membase = (u8 *)UART1_BASE;
	s.irq = IRQ_UART1;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS1 setup failed!\n");
	}
	s.line = 2;
	s.membase = (u8 *)UART2_BASE;
	s.irq = IRQ_UART2;

	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS2 setup failed!\n");
	}

	s.line = 3;
	s.membase = (u8 *)UART3_BASE;
	s.irq = IRQ_UART3;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS3 setup failed!\n");
	}
#endif
}

void __init plat_mem_setup(void)
{
	__asm__ (
		"li    $2, 0xa9000000 \n\t"
		"mtc0  $2, $5, 4      \n\t"
		"nop                  \n\t"
		::"r"(2));

	/* IO/MEM resources. Which will be the addtion value in `inX' and
	 * `outX' macros defined in asm/io.h */
	set_io_port_base(0);
	ioport_resource.start	= 0x00000000;
	ioport_resource.end	= 0xffffffff;
	iomem_resource.start	= 0x00000000;
	iomem_resource.end	= 0xffffffff;

	jz_reset_init();

	jz_soc_setup();
	jz_serial_setup();
	jz_board_setup();

#ifdef CONFIG_PM
	jz_pm_init();
#endif
}


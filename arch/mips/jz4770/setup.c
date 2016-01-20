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

#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/ioport.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/tty.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>
#include <asm/pgtable.h>
#include <asm/prom.h>
#include <asm/time.h>
#include <asm/page.h>

#ifdef CONFIG_PC_KEYB
#include <asm/keyboard.h>
#endif

static void jz_halt(void)
{
	printk(KERN_NOTICE "\n** You can safely turn off the power\n");

	while (1)
		__asm__(".set\tmips3\n\t"
	                "wait\n\t"
			".set\tmips0");
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

	_machine_halt = jz_halt;
	__dt_setup_arch(__dtb_start);
}

/*
 * We have seen MMC DMA transfers read corrupted data from SDRAM when a burst
 * interval ends at physical address 0x10000000. To avoid this problem, we
 * remove the final page of low memory from the memory map.
 */
void __init jz4770_reserve_unsafe_for_dma(void)
{
	int i;
	for (i = 0; i < boot_mem_map.nr_map; i++) {
		struct boot_mem_map_entry *entry = boot_mem_map.map + i;

		if (entry->type != BOOT_MEM_RAM)
			continue;

		if (entry->addr + entry->size != 0x10000000)
			continue;

		entry->size -= PAGE_SIZE;
		break;
	}
}

void __init device_tree_init(void)
{
	if (!initial_boot_params)
		return;

	unflatten_and_copy_device_tree();
}

void __init arch_init_irq(void)
{
	irqchip_init();
}

void __init plat_time_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
}

static int __init populate_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	return 0;
}
arch_initcall(populate_machine);

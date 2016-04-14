#include <linux/init.h>

#include <asm/bootinfo.h>
#include <asm/fw/fw.h>

void __init prom_init(void)
{
	mips_machtype = MACH_INGENIC_JZ4770;
	fw_init_cmdline();
}

void __init prom_free_prom_memory(void)
{
}

/*
 *  Copyright (C) 2015, Paul Cercueil <paul@crapouillou.net>
 *	JZ4770 SoC power management support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General	 Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/suspend.h>

#include <asm/reboot.h>

static void jz4770_wait(void)
{
	__asm__(".set push;\n"
		".set mips3;\n"
		"wait;\n"
		".set pop;\n"
	);
}

static void jz4770_halt(void)
{
	while (1)
		jz4770_wait();
}

static int __init jz4770_halt_init(void)
{
	_machine_halt = jz4770_halt;
	return 0;
}
core_initcall(jz4770_halt_init);

#ifdef CONFIG_PM_SUSPEND
static int jz4740_pm_enter(suspend_state_t state)
{
	jz4770_wait();
	return 0;
}

static const struct platform_suspend_ops jz4740_pm_ops = {
	.valid		= suspend_valid_only_mem,
	.enter		= jz4740_pm_enter,
};

static int __init jz4740_pm_init(void)
{
	suspend_set_ops(&jz4740_pm_ops);
	return 0;

}
late_initcall(jz4740_pm_init);
#endif /* CONFIG_PM_SUSPEND */

/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2012, Paul Cercueil <paul@crapouillou.net>
 *  JZ4770 SoC clock support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/io.h>

#include <asm/mach-jz4770/base.h>

#define CPM_CPCCR_OFFSET 0x00

#define TCU_TCSR_OFFSET(timer) (0x3C + (timer) * 0x10) /* Timer Control reg. */

#define WDT_TCSR_OFFSET 0xC
#define OST_OSTCSR_OFFSET 0xEC

#define CLK_PARENT_EXT	BIT(2)
#define CLK_PARENT_RTC	BIT(1)
#define CLK_PARENT_PCLK	BIT(0)

#define CPCCR_CE	(1 << 22)
#define CPCCR_PCS	(1 << 21)

int jz_clk_init(void)
{
	u32 reg;
	size_t i;
	void __iomem *tcu_base, __iomem *wdt_tcsr,
	     __iomem *cpm_base, __iomem *ost_tcsr;

	cpm_base = ioremap_nocache(JZ4770_CPM_BASE_ADDR, 4);
	if (!cpm_base)
		return -EBUSY;

	tcu_base = ioremap_nocache(JZ4770_TCU_BASE_ADDR, 0x100);
	if (!tcu_base)
		return -EBUSY;

	wdt_tcsr = ioremap_nocache(JZ4770_WDT_BASE_ADDR + WDT_TCSR_OFFSET, 2);
	if (!wdt_tcsr)
		return -EBUSY;

	ost_tcsr = ioremap_nocache(JZ4770_OST_BASE_ADDR + OST_OSTCSR_OFFSET, 2);
	if (!ost_tcsr)
		return -EBUSY;

	writew(CLK_PARENT_RTC, wdt_tcsr + TCU_TCSR_OFFSET(0));
	writew(CLK_PARENT_EXT, ost_tcsr + TCU_TCSR_OFFSET(0));

	for (i = 0; i < 8; i++)
		writew(CLK_PARENT_EXT, tcu_base + TCU_TCSR_OFFSET(i));

	reg = readl(cpm_base + CPM_CPCCR_OFFSET);

	reg |= CPCCR_CE;
	reg &= ~CPCCR_PCS;
	writel(reg, cpm_base + CPM_CPCCR_OFFSET);

	iounmap(cpm_base);
	iounmap(tcu_base);
	iounmap(wdt_tcsr);
	iounmap(ost_tcsr);
	return 0;
}

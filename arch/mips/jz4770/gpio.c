/*
 * JZ4770 platform GPIO support
 *
 * Original JZ4770 support by hlguo <hlguo@ingenic.cn>
 * Copyright (C) Ingenic Semiconductor Inc.
 *
 * JZ4740 platform GPIO support.
 * Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 * Stiched the JZ4740 and JZ4770 code together.
 * Copyright (C) 2012, Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/export.h>
#include <linux/pinctrl/consumer.h>

#include <asm/mach-jz4770/gpio.h>

#include "gpio.h"

void jz_gpio_enable_pullup(unsigned gpio)
{
	__gpio_enable_pull(gpio);
}
EXPORT_SYMBOL_GPL(jz_gpio_enable_pullup);

void jz_gpio_disable_pullup(unsigned gpio)
{
	__gpio_disable_pull(gpio);
}
EXPORT_SYMBOL_GPL(jz_gpio_disable_pullup);

int gpio_to_irq(unsigned gpio)
{
	return __gpio_to_irq(gpio);
}
EXPORT_SYMBOL_GPL(gpio_to_irq);

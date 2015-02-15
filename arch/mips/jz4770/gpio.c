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

#include <asm/gpio.h>

int gpio_to_irq(unsigned gpio)
{
	return __gpio_to_irq(gpio);
}
EXPORT_SYMBOL_GPL(gpio_to_irq);

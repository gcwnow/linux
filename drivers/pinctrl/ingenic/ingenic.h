/*
 * JZ4780 pinctrl driver
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef PINCTRL_INGENIC_H
#define PINCTRL_INGENIC_H

#include <linux/compiler.h>
#include <linux/types.h>

struct platform_device;

struct ingenic_pinctrl_ops {
	unsigned nb_functions;

	void (*set_function)(void __iomem *base,
			unsigned offset, unsigned function);
	void (*set_gpio)(void __iomem *base, unsigned offset, bool output);
	int  (*get_bias)(void __iomem *base, unsigned offset);
	void (*set_bias)(void __iomem *base, unsigned offset, bool enable);
	void (*gpio_set_value)(void __iomem *base, unsigned offset, int value);
	int  (*gpio_get_value)(void __iomem *base, unsigned offset);
	u32  (*irq_read)(void __iomem *base);
	void (*irq_mask)(void __iomem *base, unsigned irq, bool mask);
	void (*irq_ack)(void __iomem *base, unsigned irq);
	void (*irq_set_type)(void __iomem *base, unsigned irq, unsigned type);
};

extern struct ingenic_pinctrl_ops ingenic_pinctrl_ops;

#endif /* PINCTRL_INGENIC_H */

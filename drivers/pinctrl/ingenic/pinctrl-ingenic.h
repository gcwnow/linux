/*
 * Ingenic SoCs pinctrl driver
 *
 * Copyright (c) 2013 Imagination Technologies
 * Copyright (c) 2016 Paul Cercueil <paul@crapouillou.net>
 *
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef PINCTRL_INGENIC_H
#define PINCTRL_INGENIC_H

#include <linux/compiler.h>
#include <linux/types.h>

struct platform_device;

struct ingenic_pinctrl_ops {
	unsigned int nb_functions;

	void (*set_function)(void __iomem *base,
			unsigned int offset, unsigned int function);
	void (*set_gpio)(void __iomem *base, unsigned int offset, bool output);
	int  (*get_bias)(void __iomem *base, unsigned int offset);
	void (*set_bias)(void __iomem *base, unsigned int offset, bool enable);
	void (*gpio_set_value)(void __iomem *base,
			unsigned int offset, int value);
	int  (*gpio_get_value)(void __iomem *base, unsigned int offset);
	u32  (*irq_read)(void __iomem *base);
	void (*irq_mask)(void __iomem *base, unsigned int irq, bool mask);
	void (*irq_ack)(void __iomem *base, unsigned int irq);
	void (*irq_set_type)(void __iomem *base,
			unsigned int irq, unsigned int type);
};

int ingenic_pinctrl_probe(struct platform_device *pdev,
		const struct ingenic_pinctrl_ops *ops);

#endif /* PINCTRL_INGENIC_H */

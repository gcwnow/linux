/*
 * JZ4740 SoC TCU IRQ driver
 *
 * Copyright (c) 2016 Paul Cercueil <paul@crapouillou.net>
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

#include <linux/bitops.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define REG_FLAG	0x00
#define REG_FLAG_SET	0x04
#define REG_FLAG_CLEAR	0x08
#define REG_MASK	0x10
#define REG_MASK_SET	0x14
#define REG_MASK_CLEAR	0x18

static void ingenic_tcu_intc_cascade(struct irq_desc *desc)
{
	struct irq_chip *irq_chip = irq_data_get_irq_chip(&desc->irq_data);
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	struct irq_chip_generic *gc = irq_get_domain_generic_chip(domain, 0);
	uint32_t irq_reg, irq_mask;
	unsigned i;

	irq_reg = irq_reg_readl(gc, REG_FLAG);
	irq_mask = irq_reg_readl(gc, REG_MASK);

	chained_irq_enter(irq_chip, desc);

	irq_reg &= ~irq_mask;

	for (i = 0; i < 32; i++) {
		if (irq_reg & BIT(i))
			generic_handle_irq(irq_linear_revmap(domain, i));
	}

	chained_irq_exit(irq_chip, desc);
}

static int __init ingenic_tcu_intc_of_init(struct device_node *node,
	struct device_node *parent)
{
	struct irq_domain *domain;
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	void __iomem *base;
	int err, i, num_parent_irqs;
	unsigned int parent_irqs[3];

	num_parent_irqs = of_property_count_elems_of_size(
			node, "interrupts", 4);
	if (num_parent_irqs < 0 || num_parent_irqs > ARRAY_SIZE(parent_irqs))
		return -EINVAL;

	base = of_iomap(node, 0);
	if (!base)
		return -ENODEV;

	domain = irq_domain_add_linear(node, 32, &irq_generic_chip_ops, NULL);
	if (!domain) {
		err = -ENOMEM;
		goto out_unmap_base;
	}

	err = irq_alloc_domain_generic_chips(domain, 32, 1, "TCU",
			handle_level_irq, 0, IRQ_NOPROBE | IRQ_LEVEL, 0);
	if (err)
		goto out_domain_remove;

	gc = irq_get_domain_generic_chip(domain, 0);
	ct = gc->chip_types;

	gc->wake_enabled = IRQ_MSK(32);
	gc->reg_base = base;

	ct->regs.disable = REG_MASK_SET;
	ct->regs.enable = REG_MASK_CLEAR;
	ct->regs.ack = REG_FLAG_CLEAR;
	ct->chip.irq_unmask = irq_gc_unmask_enable_reg;
	ct->chip.irq_mask = irq_gc_mask_disable_reg;
	ct->chip.irq_mask_ack = irq_gc_mask_disable_reg_and_ack;

	/* Mask all IRQs by default */
	irq_reg_writel(gc, IRQ_MSK(32), REG_MASK_SET);

	for (i = 0; i < num_parent_irqs; i++) {
		parent_irqs[i] = irq_of_parse_and_map(node, i);
		if (!parent_irqs[i]) {
			err = -EINVAL;
			goto out_unmap_irqs;
		}

		irq_set_chained_handler_and_data(parent_irqs[i],
				ingenic_tcu_intc_cascade, domain);
	}

	return 0;

out_unmap_irqs:
	for (; i > 0; i--)
		irq_dispose_mapping(parent_irqs[i - 1]);
out_domain_remove:
	irq_domain_remove(domain);
out_unmap_base:
	iounmap(base);
	return err;
}
IRQCHIP_DECLARE(jz4740_tcu_intc, "ingenic,jz4740-tcu-intc",
		ingenic_tcu_intc_of_init);
IRQCHIP_DECLARE(jz4770_tcu_intc, "ingenic,jz4770-tcu-intc",
		ingenic_tcu_intc_of_init);
IRQCHIP_DECLARE(jz4780_tcu_intc, "ingenic,jz4780-tcu-intc",
		ingenic_tcu_intc_of_init);

// SPDX-License-Identifier: GPL-2.0
/*
 * JZ47xx SoCs TCU IRQ driver
 * Copyright (C) 2018 Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/bitops.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>

#include <linux/mfd/syscon/ingenic-tcu.h>

static void ingenic_tcu_intc_cascade(struct irq_desc *desc)
{
	struct irq_chip *irq_chip = irq_data_get_irq_chip(&desc->irq_data);
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	struct irq_chip_generic *gc = irq_get_domain_generic_chip(domain, 0);
	struct regmap *map = gc->private;
	uint32_t irq_reg, irq_mask;
	unsigned int i;

	regmap_read(map, TCU_REG_TFR, &irq_reg);
	regmap_read(map, TCU_REG_TMR, &irq_mask);

	chained_irq_enter(irq_chip, desc);

	irq_reg &= ~irq_mask;

	for (i = 0; i < 32; i++) {
		if (irq_reg & BIT(i))
			generic_handle_irq(irq_linear_revmap(domain, i));
	}

	chained_irq_exit(irq_chip, desc);
}

static void ingenic_tcu_gc_unmask_enable_reg(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct irq_chip_type *ct = irq_data_get_chip_type(d);
	struct regmap *map = gc->private;
	u32 mask = d->mask;

	irq_gc_lock(gc);
	regmap_write(map, ct->regs.ack, mask);
	regmap_write(map, ct->regs.enable, mask);
	*ct->mask_cache |= mask;
	irq_gc_unlock(gc);
}

static void ingenic_tcu_gc_mask_disable_reg(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct irq_chip_type *ct = irq_data_get_chip_type(d);
	struct regmap *map = gc->private;
	u32 mask = d->mask;

	irq_gc_lock(gc);
	regmap_write(map, ct->regs.disable, mask);
	*ct->mask_cache &= ~mask;
	irq_gc_unlock(gc);
}

static void ingenic_tcu_gc_mask_disable_reg_and_ack(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct irq_chip_type *ct = irq_data_get_chip_type(d);
	struct regmap *map = gc->private;
	u32 mask = d->mask;

	irq_gc_lock(gc);
	regmap_write(map, ct->regs.ack, mask);
	regmap_write(map, ct->regs.disable, mask);
	irq_gc_unlock(gc);
}

static int __init ingenic_tcu_intc_of_init(struct device_node *node,
	struct device_node *parent)
{
	struct irq_domain *domain;
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	int err, i, num_parent_irqs;
	unsigned int parent_irqs[3];
	struct regmap *map;

	num_parent_irqs = of_property_count_elems_of_size(
			node, "interrupts", 4);
	if (num_parent_irqs < 0 || num_parent_irqs > ARRAY_SIZE(parent_irqs))
		return -EINVAL;

	map = syscon_node_to_regmap(node->parent);
	if (IS_ERR(map))
		return PTR_ERR(map);

	domain = irq_domain_add_linear(node, 32, &irq_generic_chip_ops, NULL);
	if (!domain)
		return -ENOMEM;

	err = irq_alloc_domain_generic_chips(domain, 32, 1, "TCU",
			handle_level_irq, 0, IRQ_NOPROBE | IRQ_LEVEL, 0);
	if (err)
		goto out_domain_remove;

	gc = irq_get_domain_generic_chip(domain, 0);
	ct = gc->chip_types;

	gc->wake_enabled = IRQ_MSK(32);
	gc->private = map;

	ct->regs.disable = TCU_REG_TMSR;
	ct->regs.enable = TCU_REG_TMCR;
	ct->regs.ack = TCU_REG_TFCR;
	ct->chip.irq_unmask = ingenic_tcu_gc_unmask_enable_reg;
	ct->chip.irq_mask = ingenic_tcu_gc_mask_disable_reg;
	ct->chip.irq_mask_ack = ingenic_tcu_gc_mask_disable_reg_and_ack;
	ct->chip.flags = IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_SKIP_SET_WAKE;

	/* Mask all IRQs by default */
	regmap_write(map, TCU_REG_TMSR, IRQ_MSK(32));

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
	return err;
}

/* We only probe via devicetree, no need for a platform driver */
IRQCHIP_DECLARE(jz4740_tcu_intc, "ingenic,jz4740-tcu-intc",
		ingenic_tcu_intc_of_init);
IRQCHIP_DECLARE(jz4770_tcu_intc, "ingenic,jz4770-tcu-intc",
		ingenic_tcu_intc_of_init);
IRQCHIP_DECLARE(jz4780_tcu_intc, "ingenic,jz4780-tcu-intc",
		ingenic_tcu_intc_of_init);

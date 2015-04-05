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

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "core.h"
#include "pinconf.h"

struct jz4780_pinctrl;

struct jz4780_pinctrl_pin {
	struct jz4780_gpio_chip *gpio_chip;
	unsigned idx;
	unsigned func;
	unsigned long bias;
};

struct jz4780_pinctrl_group {
	const char *name;
	struct device_node *of_node;

	unsigned num_pins;
	struct jz4780_pinctrl_pin *pins;
	unsigned *pin_indices;
};

struct jz4780_pinctrl_func {
	const char *name;
	struct device_node *of_node;

	unsigned num_groups;
	struct jz4780_pinctrl_group **groups;
	const char **group_names;
};

struct jz4780_gpio_chip {
	char name[3];
	unsigned idx;
	struct gpio_chip gc;
	struct jz4780_pinctrl *pinctrl;
	uint32_t pull_ups;
	uint32_t pull_downs;
	unsigned int irq;
	struct irq_domain *irq_domain;
	struct pinctrl_gpio_range grange;
};

struct jz4780_pinctrl {
	struct device *dev;
	void __iomem *io_base;
	uint32_t base;
	struct pinctrl_dev *pctl;
	struct pinctrl_pin_desc *pdesc;

	unsigned num_gpio_chips;
	struct jz4780_gpio_chip *gpio_chips;

	unsigned num_groups;
	struct jz4780_pinctrl_group *groups;

	unsigned num_funcs;
	struct jz4780_pinctrl_func *funcs;
};

#define gc_to_jzgc(gpiochip) \
	container_of(gpiochip, struct jz4780_gpio_chip, gc)

/* GPIO port register offsets */
#define GPIO_PIN	0x00
#define GPIO_INT	0x10
#define GPIO_INTS	0x14
#define GPIO_INTC	0x18
#define GPIO_MSK	0x20
#define GPIO_MSKS	0x24
#define GPIO_MSKC	0x28
#define GPIO_PAT1	0x30
#define GPIO_PAT1S	0x34
#define GPIO_PAT1C	0x38
#define GPIO_PAT0	0x40
#define GPIO_PAT0S	0x44
#define GPIO_PAT0C	0x48
#define GPIO_FLG	0x50
#define GPIO_FLGC	0x58
#define GPIO_PEN	0x70
#define GPIO_PENS	0x74
#define GPIO_PENC	0x78
#define GPIO_REGS_SIZE	0x100

#define PINS_PER_GPIO_PORT 32

static struct jz4780_pinctrl_group *find_group_by_of_node(
		struct jz4780_pinctrl *jzpc, struct device_node *np)
{
	int i;
	for (i = 0; i < jzpc->num_groups; i++) {
		if (jzpc->groups[i].of_node == np)
			return &jzpc->groups[i];
	}
	return NULL;
}

static struct jz4780_pinctrl_func *find_func_by_of_node(
		struct jz4780_pinctrl *jzpc, struct device_node *np)
{
	int i;
	for (i = 0; i < jzpc->num_funcs; i++) {
		if (jzpc->funcs[i].of_node == np)
			return &jzpc->funcs[i];
	}
	return NULL;
}

static u32 jz4780_gpio_readl(struct jz4780_gpio_chip *jzgc, unsigned offset)
{
	void __iomem *base = jzgc->pinctrl->io_base +
		(jzgc->idx * GPIO_REGS_SIZE);
	return readl(base + offset);
}

static void jz4780_gpio_writel(struct jz4780_gpio_chip *jzgc, u32 value,
		unsigned offset)
{
	void __iomem *base = jzgc->pinctrl->io_base +
		(jzgc->idx * GPIO_REGS_SIZE);
	writel(value, base + offset);
}

static void jz4780_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct jz4780_gpio_chip *jzgc = gc_to_jzgc(gc);

	if (value)
		jz4780_gpio_writel(jzgc, 1 << offset, GPIO_PAT0S);
	else
		jz4780_gpio_writel(jzgc, 1 << offset, GPIO_PAT0C);
}

static int jz4780_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct jz4780_gpio_chip *jzgc = gc_to_jzgc(gc);
	return (jz4780_gpio_readl(jzgc, GPIO_PIN) >> offset) & 0x1;
}

static int jz4780_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	return pinctrl_gpio_direction_input(gc->base + offset);
}

static int jz4780_gpio_direction_output(struct gpio_chip *gc, unsigned offset,
					int value)
{
	jz4780_gpio_set(gc, offset, value);
	return pinctrl_gpio_direction_output(gc->base + offset);
}

static int jz4780_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct jz4780_gpio_chip *jzgc = gc_to_jzgc(gc);
	int virq;

	if (!jzgc->irq_domain)
		return -ENXIO;

	virq = irq_create_mapping(jzgc->irq_domain, offset);
	return (virq) ? : -ENXIO;
}

static void jz4780_gpio_irq_mask(struct irq_data *irqd)
{
	struct jz4780_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);
	jz4780_gpio_writel(jzgc, 1 << irqd->hwirq, GPIO_MSKS);
}

static void jz4780_gpio_irq_unmask(struct irq_data *irqd)
{
	struct jz4780_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);
	jz4780_gpio_writel(jzgc, 1 << irqd->hwirq, GPIO_MSKC);
}

static void jz4780_gpio_irq_ack(struct irq_data *irqd)
{
	struct jz4780_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);
	unsigned pin;

	if (irqd_get_trigger_type(irqd) == IRQ_TYPE_EDGE_BOTH) {
		/*
		 * Switch to an interrupt for the opposite edge to the one that
		 * triggered the interrupt being ACKed.
		 */
		pin = jz4780_gpio_readl(jzgc, GPIO_PIN);
		pin &= 1 << irqd->hwirq;

		jz4780_gpio_writel(jzgc, 1 << irqd->hwirq,
				   pin ? GPIO_PAT0C : GPIO_PAT0S);
	}

	jz4780_gpio_writel(jzgc, 1 << irqd->hwirq, GPIO_FLGC);
}

static int jz4780_gpio_irq_set_type(struct irq_data *irqd, unsigned int type)
{
	struct jz4780_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);
	unsigned pin;
	enum {
		PAT_EDGE_RISING		= 0x3,
		PAT_EDGE_FALLING	= 0x2,
		PAT_LEVEL_HIGH		= 0x1,
		PAT_LEVEL_LOW		= 0x0,
	} pat;

	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
		/*
		 * The hardware does not support interrupts on both edges. The
		 * best we can do is to set up a single-edge interrupt and then
		 * switch to the opposing edge when ACKing the interrupt.
		 */
		pin = jz4780_gpio_readl(jzgc, GPIO_PIN);
		pin &= 1 << irqd->hwirq;
		pat = pin ? PAT_EDGE_FALLING : PAT_EDGE_RISING;
		break;
	case IRQ_TYPE_EDGE_RISING:
		pat = PAT_EDGE_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		pat = PAT_EDGE_FALLING;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		pat = PAT_LEVEL_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		pat = PAT_LEVEL_LOW;
		break;
	default:
		pr_err("unsupported external interrupt type\n");
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_BOTH)
		__irq_set_handler_locked(irqd->irq, handle_edge_irq);
	else
		__irq_set_handler_locked(irqd->irq, handle_level_irq);

	jz4780_gpio_writel(jzgc, 1 << irqd->hwirq,
			   (pat & 0x2) ? GPIO_PAT1S : GPIO_PAT1C);
	jz4780_gpio_writel(jzgc, 1 << irqd->hwirq,
			   (pat & 0x1) ? GPIO_PAT0S : GPIO_PAT0C);
	jz4780_gpio_writel(jzgc, 1 << irqd->hwirq, GPIO_INTS);

	return 0;
}

static struct irq_chip jz4780_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_unmask	= jz4780_gpio_irq_unmask,
	.irq_mask	= jz4780_gpio_irq_mask,
	.irq_ack	= jz4780_gpio_irq_ack,
	.irq_set_type	= jz4780_gpio_irq_set_type,
};

static void jz4780_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct jz4780_gpio_chip *jzgc = irq_get_handler_data(irq);
	unsigned long flag, i;

	flag = jz4780_gpio_readl(jzgc, GPIO_FLG);

	for_each_set_bit(i, &flag, 32)
		generic_handle_irq(irq_find_mapping(jzgc->irq_domain, i));
}

static int jz4780_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	return jzpc->num_groups;
}

static const char *jz4780_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						 unsigned selector)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	return jzpc->groups[selector].name;
}

static int jz4780_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					 unsigned selector,
					 const unsigned **pins,
					 unsigned *num_pins)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= jzpc->num_groups)
		return -EINVAL;

	*pins = jzpc->groups[selector].pin_indices;
	*num_pins = jzpc->groups[selector].num_pins;
	return 0;
}

static int jz4780_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
					 struct device_node *np,
					 struct pinctrl_map **map,
					 unsigned *num_maps)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	struct jz4780_pinctrl_func *func;
	struct jz4780_pinctrl_group *group;
	struct pinctrl_map *new_map;
	unsigned map_num, i;

	group = find_group_by_of_node(jzpc, np);
	if (!group)
		return -EINVAL;

	func = find_func_by_of_node(jzpc, of_get_parent(np));
	if (!func)
		return -EINVAL;

	map_num = 1 + group->num_pins;
	new_map = devm_kzalloc(jzpc->dev,
				sizeof(*new_map) * map_num, GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = func->name;
	new_map[0].data.mux.group = group->name;

	for (i = 0; i < group->num_pins; i++) {
		new_map[i + 1].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[i + 1].data.configs.group_or_pin =
			jzpc->pdesc[group->pins[i].idx].name;
		new_map[i + 1].data.configs.configs = &group->pins[i].bias;
		new_map[i + 1].data.configs.num_configs = 1;
	}

	*map = new_map;
	*num_maps = map_num;
	return 0;
}

static void jz4780_pinctrl_dt_free_map(struct pinctrl_dev *pctldev,
				       struct pinctrl_map *map,
				       unsigned num_maps)
{
}

static struct pinctrl_ops jz4780_pctlops = {
	.get_groups_count = jz4780_pinctrl_get_groups_count,
	.get_group_name = jz4780_pinctrl_get_group_name,
	.get_group_pins = jz4780_pinctrl_get_group_pins,
	.dt_node_to_map = jz4780_pinctrl_dt_node_to_map,
	.dt_free_map = jz4780_pinctrl_dt_free_map,
};

static int jz4780_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	return jzpc->num_funcs;
}

static const char *jz4780_pinmux_get_function_name(struct pinctrl_dev *pctldev,
						   unsigned selector)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	return jzpc->funcs[selector].name;
}

static int jz4780_pinmux_get_function_groups(struct pinctrl_dev *pctldev,
					     unsigned selector,
					     const char * const **groups,
					     unsigned * const num_groups)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= jzpc->num_funcs)
		return -EINVAL;

	*groups = jzpc->funcs[selector].group_names;
	*num_groups = jzpc->funcs[selector].num_groups;

	return 0;
}

static int jz4780_pinmux_set_pin_fn(struct jz4780_pinctrl *jzpc,
				    struct jz4780_pinctrl_pin *pin)
{
	unsigned idx = pin->idx % PINS_PER_GPIO_PORT;

	dev_dbg(jzpc->dev, "set pin P%c%u to function %u\n",
		'A' + pin->gpio_chip->idx, idx, pin->func);

	if (pin->func > 3)
		return -EINVAL;

	jz4780_gpio_writel(pin->gpio_chip, 1 << idx, GPIO_INTC);
	jz4780_gpio_writel(pin->gpio_chip, 1 << idx, GPIO_MSKC);
	jz4780_gpio_writel(pin->gpio_chip, 1 << idx,
			(pin->func & 0x2) ? GPIO_PAT1S : GPIO_PAT1C);
	jz4780_gpio_writel(pin->gpio_chip, 1 << idx,
			(pin->func & 0x1) ? GPIO_PAT0S : GPIO_PAT0C);
	return 0;
}

static int jz4780_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned selector,
				unsigned group)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	struct jz4780_pinctrl_group *grp = &jzpc->groups[group];
	unsigned i;
	int err = 0;

	if (selector >= jzpc->num_funcs || group >= jzpc->num_groups)
		return -EINVAL;

	for (i = 0; i < grp->num_pins; i++) {
		err = jz4780_pinmux_set_pin_fn(jzpc, &grp->pins[i]);
		if (err)
			break;
	}

	return err;
}

static int jz4780_pinmux_gpio_set_direction(struct pinctrl_dev *pctldev,
					    struct pinctrl_gpio_range *range,
					    unsigned offset, bool input)
{
	struct jz4780_gpio_chip *jzgc = gc_to_jzgc(range->gc);
	unsigned idx;

	idx = offset - (jzgc->idx * PINS_PER_GPIO_PORT);

	jz4780_gpio_writel(jzgc, 1 << offset, GPIO_INTC);
	jz4780_gpio_writel(jzgc, 1 << offset, GPIO_MSKS);
	jz4780_gpio_writel(jzgc, 1 << offset, input ? GPIO_PAT1S : GPIO_PAT1C);

	return 0;
}

static struct pinmux_ops jz4780_pmxops = {
	.get_functions_count = jz4780_pinmux_get_functions_count,
	.get_function_name = jz4780_pinmux_get_function_name,
	.get_function_groups = jz4780_pinmux_get_function_groups,
	.set_mux = jz4780_pinmux_set_mux,
	.gpio_set_direction = jz4780_pinmux_gpio_set_direction,
};

static int jz4780_pinconf_get(struct pinctrl_dev *pctldev,
			      unsigned int pin, unsigned long *config)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	struct jz4780_gpio_chip *jzgc;
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned idx, pull;

	if (pin >= (jzpc->num_gpio_chips * PINS_PER_GPIO_PORT))
		return -EINVAL;
	jzgc = &jzpc->gpio_chips[pin / PINS_PER_GPIO_PORT];
	idx = pin % PINS_PER_GPIO_PORT;

	pull = !((jz4780_gpio_readl(jzgc, GPIO_PEN) >> idx) & 0x1);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (pull)
			return -EINVAL;
		break;

	case PIN_CONFIG_BIAS_PULL_UP:
		if (!pull || !(jzgc->pull_ups & (1 << idx)))
			return -EINVAL;
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (!pull || !(jzgc->pull_downs & (1 << idx)))
			return -EINVAL;
		break;

	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, 1);
	return 0;
}

static int jz4780_pinconf_set(struct pinctrl_dev *pctldev,
			      unsigned int pin, unsigned long *configs,
			      unsigned num_configs)
{
	struct jz4780_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	struct jz4780_gpio_chip *jzgc;
	unsigned idx, cfg;

	if (pin >= (jzpc->num_gpio_chips * PINS_PER_GPIO_PORT))
		return -EINVAL;
	jzgc = &jzpc->gpio_chips[pin / PINS_PER_GPIO_PORT];
	idx = pin % PINS_PER_GPIO_PORT;

	for (cfg = 0; cfg < num_configs; cfg++) {
		dev_dbg(jzpc->dev, "%s %u %lu\n", __func__, pin, configs[cfg]);

		switch (pinconf_to_config_param(configs[cfg])) {
		case PIN_CONFIG_BIAS_DISABLE:
			jz4780_gpio_writel(jzgc, 1 << idx, GPIO_PENS);
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
			if (!(jzgc->pull_ups & (1 << idx)))
				return -EINVAL;
			jz4780_gpio_writel(jzgc, 1 << idx, GPIO_PENC);
			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			if (!(jzgc->pull_downs & (1 << idx)))
				return -EINVAL;
			jz4780_gpio_writel(jzgc, 1 << idx, GPIO_PENC);
			break;

		default:
			return -ENOTSUPP;
		}
	}

	return 0;
}

static struct pinconf_ops jz4780_confops = {
	.is_generic = true,
	.pin_config_get = jz4780_pinconf_get,
	.pin_config_set = jz4780_pinconf_set,
};

static int jz4780_pinctrl_parse_dt_gpio(struct jz4780_pinctrl *jzpc,
					struct jz4780_gpio_chip *jzgc,
					struct device_node *np)
{
	unsigned gpio;
	int err, irq;

	jzgc->pinctrl = jzpc;
	snprintf(jzgc->name, sizeof(jzgc->name), "P%c", 'A' + jzgc->idx);

	jzgc->gc.base = jzpc->base + (jzgc->idx * PINS_PER_GPIO_PORT);
	jzgc->gc.ngpio = PINS_PER_GPIO_PORT;
	jzgc->gc.dev = jzpc->dev;
	jzgc->gc.of_node = np;
	jzgc->gc.label = np->name;
	jzgc->gc.owner = THIS_MODULE;

	jzgc->gc.set = jz4780_gpio_set;
	jzgc->gc.get = jz4780_gpio_get;
	jzgc->gc.direction_input = jz4780_gpio_direction_input;
	jzgc->gc.direction_output = jz4780_gpio_direction_output;
	jzgc->gc.to_irq = jz4780_gpio_to_irq;

	if (of_property_read_u32_index(np, "ingenic,pull-ups", 0,
				&jzgc->pull_ups))
		jzgc->pull_ups = 0;
	if (of_property_read_u32_index(np, "ingenic,pull-downs", 0,
				&jzgc->pull_downs))
		jzgc->pull_downs = 0;

	if (jzgc->pull_ups & jzgc->pull_downs) {
		dev_err(jzpc->dev, "GPIO port %c has overlapping pull ups & pull downs\n",
			'A' + jzgc->idx);
		return -EINVAL;
	}

	err = gpiochip_add(&jzgc->gc);
	if (err)
		return err;

	if (!of_find_property(np, "interrupt-controller", NULL))
		return 0;

	jzgc->irq = irq_of_parse_and_map(np, 0);
	if (!jzgc->irq)
		return -EINVAL;

	/* register IRQ domain */
	jzgc->irq_domain = irq_domain_add_linear(np, jzgc->gc.ngpio,
						 &irq_domain_simple_ops, NULL);
	if (!jzgc->irq_domain)
		return -EINVAL;

	for (gpio = 0; gpio < jzgc->gc.ngpio; gpio++) {
		irq = irq_create_mapping(jzgc->irq_domain, gpio);

		irq_set_chip_data(irq, jzgc);
		irq_set_chip_and_handler(irq, &jz4780_gpio_irq_chip,
					 handle_level_irq);
	}

	irq_set_chained_handler(jzgc->irq, jz4780_gpio_irq_handler);
	irq_set_handler_data(jzgc->irq, jzgc);

	return 0;
}

static int find_gpio_chip_by_of_node(struct gpio_chip *chip, void *data)
{
	return chip->of_node == data;
}

static int jz4780_pinctrl_parse_dt_pincfg(struct jz4780_pinctrl *jzpc,
					  struct jz4780_pinctrl_pin *pin,
					  phandle cfg_handle)
{
	struct device_node *cfg_node;
	unsigned long *configs;
	unsigned int num_configs, i;
	enum pin_config_param conf_param;
	int err;

	cfg_node = of_find_node_by_phandle(cfg_handle);
	if (!cfg_node)
		return -EINVAL;

	err = pinconf_generic_parse_dt_config(cfg_node, &configs,
			&num_configs);
	if (err)
		return err;

	for (i = 0; i < num_configs; i++) {
		conf_param = pinconf_to_config_param(configs[i]);
		switch (conf_param) {
		case PIN_CONFIG_BIAS_DISABLE:
		case PIN_CONFIG_BIAS_PULL_UP:
		case PIN_CONFIG_BIAS_PULL_DOWN:
			pin->bias = (unsigned long)conf_param;
			break;
		default:
			dev_err(jzpc->dev, "unhandled pinconf parameter %u",
					(unsigned)conf_param);
			err = -EINVAL;
			goto out;
		}
	}

out:
	kfree(configs);
	return err;
}

static int jz4780_pinctrl_parse_dt_func(struct jz4780_pinctrl *jzpc,
					struct device_node *np,
					unsigned *ifunc, unsigned *igroup)
{
	struct jz4780_pinctrl_func *func;
	struct jz4780_pinctrl_group *grp;
	struct device_node *group_node, *gpio_node;
	struct gpio_chip *gpio_chip;
	phandle gpio_handle, cfg_handle;
	struct property *pp;
	__be32 *plist;
	unsigned i, j;
	int err;
	const unsigned vals_per_pin = 4;

	func = &jzpc->funcs[(*ifunc)++];
	func->of_node = np;
	func->name = np->name;

	func->num_groups = of_get_child_count(np);
	func->groups = devm_kzalloc(jzpc->dev, sizeof(*func->groups) *
			func->num_groups, GFP_KERNEL);
	func->group_names = devm_kzalloc(jzpc->dev,
			sizeof(*func->group_names) * func->num_groups,
			GFP_KERNEL);
	if (!func->groups || !func->group_names)
		return -ENOMEM;

	i = 0;
	for_each_child_of_node(np, group_node) {
		pp = of_find_property(group_node, "ingenic,pins", NULL);
		if (!pp)
			return -EINVAL;
		if ((pp->length / sizeof(__be32)) % vals_per_pin)
			return -EINVAL;

		grp = &jzpc->groups[(*igroup)++];
		grp->of_node = group_node;
		grp->name = group_node->name;
		grp->num_pins = (pp->length / sizeof(__be32)) / vals_per_pin;
		grp->pins = devm_kzalloc(jzpc->dev, sizeof(*grp->pins) *
				grp->num_pins, GFP_KERNEL);
		grp->pin_indices = devm_kzalloc(jzpc->dev,
				sizeof(*grp->pin_indices) * grp->num_pins,
				GFP_KERNEL);
		if (!grp->pins)
			return -EINVAL;

		plist = pp->value;
		for (j = 0; j < grp->num_pins; j++) {
			gpio_handle = be32_to_cpup(plist++);
			grp->pins[j].idx = be32_to_cpup(plist++);
			grp->pins[j].func = be32_to_cpup(plist++);
			cfg_handle = be32_to_cpup(plist++);

			gpio_node = of_find_node_by_phandle(gpio_handle);
			if (!gpio_node)
				return -EINVAL;

			gpio_chip = gpiochip_find(gpio_node,
					find_gpio_chip_by_of_node);
			if (!gpio_chip)
				return -EINVAL;

			grp->pins[j].gpio_chip = gc_to_jzgc(gpio_chip);

			err = jz4780_pinctrl_parse_dt_pincfg(jzpc,
					&grp->pins[j], cfg_handle);
			if (err)
				return err;

			grp->pins[j].idx += grp->pins[j].gpio_chip->idx *
				PINS_PER_GPIO_PORT;
			grp->pin_indices[j] = grp->pins[j].idx;
		}

		func->groups[i] = grp;
		func->group_names[i] = grp->name;
		i++;
	}

	return 0;
}

static int jz4780_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jz4780_pinctrl *jzpc;
	struct jz4780_gpio_chip *jzgc;
	struct pinctrl_desc *pctl_desc;
	struct device_node *np, *group_node;
	unsigned i, j;
	int err;

	if (!dev->of_node) {
		dev_err(dev, "device tree node not found\n");
		return -ENODEV;
	}

	jzpc = devm_kzalloc(dev, sizeof(*jzpc), GFP_KERNEL);
	if (!jzpc) {
		dev_err(dev, "failed to allocate private data\n");
		return -ENOMEM;
	}
	jzpc->dev = dev;
	platform_set_drvdata(pdev, jzpc);

	jzpc->io_base = of_iomap(dev->of_node, 0);
	if (!jzpc->io_base) {
		dev_err(dev, "failed to map IO memory\n");
		return -ENXIO;
	}

	jzpc->base = 0;
	of_property_read_u32(dev->of_node, "base", &jzpc->base);

	/* count GPIO chips, pin groups & functions */
	jzpc->num_gpio_chips = jzpc->num_groups = jzpc->num_funcs = 0;
	for_each_child_of_node(dev->of_node, np) {
		if (of_find_property(np, "gpio-controller", NULL)) {
			jzpc->num_gpio_chips++;
			continue;
		}

		if (of_find_property(np, "bias-disable", NULL) ||
			of_find_property(np, "bias-pull-up", NULL) ||
			of_find_property(np, "bias-pull-down", NULL)) {
			continue;
		}

		if (!of_get_child_count(np)) {
			dev_err(dev, "function with no groups %s\n",
					np->full_name);
			return -EINVAL;
		}

		jzpc->num_funcs++;

		for_each_child_of_node(np, group_node) {
			if (!of_find_property(group_node, "ingenic,pins",
						NULL)) {
				dev_err(dev, "invalid DT node %s\n",
						group_node->full_name);
				return -EINVAL;
			}
			jzpc->num_groups++;
		}
	}

	/* allocate memory for GPIO chips, pin groups & functions */
	jzpc->gpio_chips = devm_kzalloc(jzpc->dev, sizeof(*jzpc->gpio_chips) *
			jzpc->num_gpio_chips, GFP_KERNEL);
	jzpc->groups = devm_kzalloc(jzpc->dev, sizeof(*jzpc->groups) *
			jzpc->num_groups, GFP_KERNEL);
	jzpc->funcs = devm_kzalloc(jzpc->dev, sizeof(*jzpc->funcs) *
			jzpc->num_funcs, GFP_KERNEL);
	pctl_desc = devm_kzalloc(&pdev->dev, sizeof(*pctl_desc), GFP_KERNEL);
	if (!jzpc->gpio_chips || !jzpc->groups || !jzpc->funcs || !pctl_desc)
		return -ENOMEM;

	/* register GPIO chips, count pins */
	i = 0;
	for_each_child_of_node(dev->of_node, np) {
		if (!of_find_property(np, "gpio-controller", NULL))
			continue;

		jzpc->gpio_chips[i].idx = i;

		err = jz4780_pinctrl_parse_dt_gpio(jzpc,
				&jzpc->gpio_chips[i++], np);
		if (err) {
			dev_err(dev, "failed to register GPIO chip: %d\n", err);
			return err;
		}

		pctl_desc->npins += PINS_PER_GPIO_PORT;
	}

	/* fill in pin descriptions */
	pctl_desc->name = dev_name(dev);
	pctl_desc->owner = THIS_MODULE;
	pctl_desc->pins = jzpc->pdesc = devm_kzalloc(&pdev->dev,
			sizeof(*jzpc->pdesc) * pctl_desc->npins, GFP_KERNEL);
	if (!jzpc->pdesc)
		return -ENOMEM;

	for (i = 0; i < pctl_desc->npins; i++) {
		jzpc->pdesc[i].number = i;
		jzpc->pdesc[i].name = kasprintf(GFP_KERNEL, "P%c%d",
						'A' + (i / PINS_PER_GPIO_PORT),
						i % PINS_PER_GPIO_PORT);
	}

	/* parse pin functions */
	i = j = 0;
	for_each_child_of_node(dev->of_node, np) {
		if (of_find_property(np, "gpio-controller", NULL))
			continue;

		if (of_find_property(np, "bias-disable", NULL) ||
			of_find_property(np, "bias-pull-up", NULL) ||
			of_find_property(np, "bias-pull-down", NULL)) {
			continue;
		}

		err = jz4780_pinctrl_parse_dt_func(jzpc, np, &i, &j);
		if (err) {
			dev_err(dev, "failed to parse function %s\n",
					np->full_name);
			return err;
		}
	}
	BUG_ON(i != jzpc->num_funcs);
	BUG_ON(j != jzpc->num_groups);

	for (i = 0; i < jzpc->num_groups; i++)
		dev_dbg(dev, "group '%s'\n", jzpc->groups[i].name);
	for (i = 0; i < jzpc->num_funcs; i++)
		dev_dbg(dev, "func '%s'\n", jzpc->funcs[i].name);

	pctl_desc->pctlops = &jz4780_pctlops;
	pctl_desc->pmxops = &jz4780_pmxops;
	pctl_desc->confops = &jz4780_confops;

	jzpc->pctl = pinctrl_register(pctl_desc, dev, jzpc);
	if (!jzpc->pctl) {
		dev_err(dev, "Failed pinctrl registration\n");
		return -EINVAL;
	}

	/* register pinctrl GPIO ranges */
	for (i = 0; i < jzpc->num_gpio_chips; i++) {
		jzgc = &jzpc->gpio_chips[i];

		jzgc->grange.name = jzgc->name;
		jzgc->grange.id = jzgc->idx;
		jzgc->grange.pin_base = jzgc->idx * PINS_PER_GPIO_PORT;
		jzgc->grange.base = jzgc->gc.base;
		jzgc->grange.npins = jzgc->gc.ngpio;
		jzgc->grange.gc = &jzgc->gc;
		pinctrl_add_gpio_range(jzpc->pctl, &jzgc->grange);
	}

	return 0;
}

static const struct of_device_id jz4780_pinctrl_dt_match[] = {
	{ .compatible = "ingenic,jz4770-pinctrl", .data = NULL },
	{ .compatible = "ingenic,jz4780-pinctrl", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, jz4780_pinctrl_dt_match);

static struct platform_driver jz4780_pinctrl_driver = {
	.probe		= jz4780_pinctrl_probe,
	.driver = {
		.name	= "jz4780-pinctrl",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(jz4780_pinctrl_dt_match),
	},
};

static int __init jz4780_pinctrl_drv_register(void)
{
	return platform_driver_register(&jz4780_pinctrl_driver);
}
postcore_initcall(jz4780_pinctrl_drv_register);

static void __exit jz4780_pinctrl_drv_unregister(void)
{
	platform_driver_unregister(&jz4780_pinctrl_driver);
}
module_exit(jz4780_pinctrl_drv_unregister);

MODULE_AUTHOR("Paul Burton <paul.burton@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 pinctrl driver");
MODULE_LICENSE("GPL v2");

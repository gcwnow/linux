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

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <dt-bindings/pinctrl/ingenic.h>

#include "../core.h"
#include "../pinconf.h"
#include "pinctrl-ingenic.h"

struct ingenic_pinctrl;

struct ingenic_pinctrl_pin {
	struct ingenic_gpio_chip *gpio_chip;
	unsigned int idx;
	unsigned int func;
	unsigned long *configs;
	unsigned int num_configs;
};

struct ingenic_pinctrl_group {
	const char *name;
	struct device_node *of_node;

	unsigned int num_pins;
	struct ingenic_pinctrl_pin *pins;
	unsigned int *pin_indices;
};

struct ingenic_pinctrl_func {
	const char *name;
	struct device_node *of_node;

	unsigned int num_groups;
	struct ingenic_pinctrl_group **groups;
	const char **group_names;
};

struct ingenic_gpio_chip {
	char name[3];
	unsigned int idx;
	void __iomem *base;
	struct gpio_chip gc;
	struct irq_chip irq_chip;
	struct ingenic_pinctrl *pinctrl;
	const struct ingenic_pinctrl_ops *ops;
	uint32_t pull_ups;
	uint32_t pull_downs;
	unsigned int irq;
	struct pinctrl_gpio_range grange;
};

struct ingenic_pinctrl {
	struct device *dev;
	uint32_t base;
	struct pinctrl_dev *pctl;
	struct pinctrl_pin_desc *pdesc;

	unsigned int num_gpio_chips;
	struct ingenic_gpio_chip *gpio_chips;

	unsigned int num_groups;
	struct ingenic_pinctrl_group *groups;

	unsigned int num_funcs;
	struct ingenic_pinctrl_func *funcs;
};

#define gc_to_jzgc(gpiochip) \
	container_of(gpiochip, struct ingenic_gpio_chip, gc)

#define PINS_PER_GPIO_PORT 32

static struct ingenic_pinctrl_group *find_group_by_of_node(
		struct ingenic_pinctrl *jzpc, struct device_node *np)
{
	int i;

	for (i = 0; i < jzpc->num_groups; i++)
		if (jzpc->groups[i].of_node == np)
			return &jzpc->groups[i];

	return NULL;
}

static struct ingenic_pinctrl_func *find_func_by_of_node(
		struct ingenic_pinctrl *jzpc, struct device_node *np)
{
	int i;

	for (i = 0; i < jzpc->num_funcs; i++)
		if (jzpc->funcs[i].of_node == np)
			return &jzpc->funcs[i];

	return NULL;
}

static void ingenic_gpio_set(struct gpio_chip *gc,
		unsigned int offset, int value)
{
	struct ingenic_gpio_chip *jzgc = gc_to_jzgc(gc);

	jzgc->ops->gpio_set_value(jzgc->base, offset, value);
}

static int ingenic_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct ingenic_gpio_chip *jzgc = gc_to_jzgc(gc);

	return jzgc->ops->gpio_get_value(jzgc->base, offset);
}

static int ingenic_gpio_direction_input(struct gpio_chip *gc,
		unsigned int offset)
{
	return pinctrl_gpio_direction_input(gc->base + offset);
}

static int ingenic_gpio_direction_output(struct gpio_chip *gc,
		unsigned int offset, int value)
{
	ingenic_gpio_set(gc, offset, value);
	return pinctrl_gpio_direction_output(gc->base + offset);
}

static void ingenic_gpio_irq_mask(struct irq_data *irqd)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irqd);
	struct ingenic_gpio_chip *jzgc = gc_to_jzgc(gc);

	jzgc->ops->irq_mask(jzgc->base, irqd->hwirq, true);
}

static void ingenic_gpio_irq_unmask(struct irq_data *irqd)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irqd);
	struct ingenic_gpio_chip *jzgc = gc_to_jzgc(gc);

	jzgc->ops->irq_mask(jzgc->base, irqd->hwirq, false);
}

static void ingenic_gpio_irq_ack(struct irq_data *irqd)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irqd);
	struct ingenic_gpio_chip *jzgc = gc_to_jzgc(gc);
	unsigned int high;
	int irq = irqd->hwirq;

	if (irqd_get_trigger_type(irqd) == IRQ_TYPE_EDGE_BOTH) {
		/*
		 * Switch to an interrupt for the opposite edge to the one that
		 * triggered the interrupt being ACKed.
		 */
		high = jzgc->ops->gpio_get_value(jzgc->base, irq);
		if (high)
			jzgc->ops->irq_set_type(jzgc->base, irq,
					IRQ_TYPE_EDGE_FALLING);
		else
			jzgc->ops->irq_set_type(jzgc->base, irq,
					IRQ_TYPE_EDGE_RISING);
	}

	jzgc->ops->irq_ack(jzgc->base, irq);
}

static int ingenic_gpio_irq_set_type(struct irq_data *irqd, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(irqd);
	struct ingenic_gpio_chip *jzgc = gc_to_jzgc(gc);

	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		break;
	default:
		pr_err("unsupported external interrupt type\n");
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_BOTH)
		irq_set_handler_locked(irqd, handle_edge_irq);
	else
		irq_set_handler_locked(irqd, handle_level_irq);

	if (type == IRQ_TYPE_EDGE_BOTH) {
		/*
		 * The hardware does not support interrupts on both edges. The
		 * best we can do is to set up a single-edge interrupt and then
		 * switch to the opposing edge when ACKing the interrupt.
		 */
		int value = jzgc->ops->gpio_get_value(jzgc->base, irqd->hwirq);

		type = value ? IRQ_TYPE_EDGE_FALLING : IRQ_TYPE_EDGE_RISING;
	}

	jzgc->ops->irq_set_type(jzgc->base, irqd->hwirq, type);
	return 0;
}

static void ingenic_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct ingenic_gpio_chip *jzgc = gc_to_jzgc(gc);
	struct irq_chip *irq_chip = irq_data_get_irq_chip(&desc->irq_data);
	unsigned long flag, i;

	chained_irq_enter(irq_chip, desc);
	flag = jzgc->ops->irq_read(jzgc->base);

	for_each_set_bit(i, &flag, 32)
		generic_handle_irq(irq_linear_revmap(gc->irqdomain, i));
	chained_irq_exit(irq_chip, desc);
}

static int ingenic_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);

	return jzpc->num_groups;
}

static const char *ingenic_pinctrl_get_group_name(
		struct pinctrl_dev *pctldev, unsigned int selector)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);

	return jzpc->groups[selector].name;
}

static int ingenic_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned int selector, const unsigned int **pins,
		unsigned int *num_pins)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= jzpc->num_groups)
		return -EINVAL;

	*pins = jzpc->groups[selector].pin_indices;
	*num_pins = jzpc->groups[selector].num_pins;
	return 0;
}

static int ingenic_pinctrl_dt_node_to_map(
		struct pinctrl_dev *pctldev, struct device_node *np,
		struct pinctrl_map **map, unsigned int *num_maps)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	struct ingenic_pinctrl_func *func;
	struct ingenic_pinctrl_group *group;
	struct pinctrl_map *new_map;
	unsigned int map_num, i;

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
		new_map[i + 1].data.configs.configs = group->pins[i].configs;
		new_map[i + 1].data.configs.num_configs =
			group->pins[i].num_configs;
	}

	*map = new_map;
	*num_maps = map_num;
	return 0;
}

static void ingenic_pinctrl_dt_free_map(struct pinctrl_dev *pctldev,
		struct pinctrl_map *map, unsigned int num_maps)
{
}

static struct pinctrl_ops ingenic_pctlops = {
	.get_groups_count = ingenic_pinctrl_get_groups_count,
	.get_group_name = ingenic_pinctrl_get_group_name,
	.get_group_pins = ingenic_pinctrl_get_group_pins,
	.dt_node_to_map = ingenic_pinctrl_dt_node_to_map,
	.dt_free_map = ingenic_pinctrl_dt_free_map,
};

static int ingenic_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);

	return jzpc->num_funcs;
}

static const char *ingenic_pinmux_get_function_name(
		struct pinctrl_dev *pctldev, unsigned int selector)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);

	return jzpc->funcs[selector].name;
}

static int ingenic_pinmux_get_function_groups(struct pinctrl_dev *pctldev,
		unsigned int selector, const char * const **groups,
		unsigned int * const num_groups)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= jzpc->num_funcs)
		return -EINVAL;

	*groups = jzpc->funcs[selector].group_names;
	*num_groups = jzpc->funcs[selector].num_groups;

	return 0;
}

static int ingenic_pinmux_set_pin_fn(struct ingenic_pinctrl *jzpc,
		struct ingenic_pinctrl_pin *pin)
{
	struct ingenic_gpio_chip *jzgc = &jzpc->gpio_chips[
		pin->idx / PINS_PER_GPIO_PORT];
	unsigned int idx = pin->idx % PINS_PER_GPIO_PORT;

	if (pin->func == JZ_PIN_MODE_GPIO) {
		dev_dbg(jzpc->dev, "set pin P%c%u to GPIO\n",
				'A' + pin->gpio_chip->idx, idx);

		jzgc->ops->set_gpio(jzgc->base, idx, false);
	} else if (pin->func < jzgc->ops->nb_functions) {
		dev_dbg(jzpc->dev, "set pin P%c%u to function %u\n",
				'A' + pin->gpio_chip->idx, idx, pin->func);

		jzgc->ops->set_function(jzgc->base, idx, pin->func);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int ingenic_pinmux_set_mux(struct pinctrl_dev *pctldev,
		unsigned int selector, unsigned int group)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	struct ingenic_pinctrl_group *grp = &jzpc->groups[group];
	unsigned int i;
	int err = 0;

	if (selector >= jzpc->num_funcs || group >= jzpc->num_groups)
		return -EINVAL;

	for (i = 0; i < grp->num_pins; i++) {
		err = ingenic_pinmux_set_pin_fn(jzpc, &grp->pins[i]);
		if (err)
			break;
	}

	return err;
}

static int ingenic_pinmux_gpio_set_direction(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range,
		unsigned int offset, bool input)
{
	struct ingenic_gpio_chip *jzgc = gc_to_jzgc(range->gc);
	unsigned int idx;

	idx = offset - (jzgc->idx * PINS_PER_GPIO_PORT);

	jzgc->ops->set_gpio(jzgc->base, idx, !input);
	return 0;
}

static struct pinmux_ops ingenic_pmxops = {
	.get_functions_count = ingenic_pinmux_get_functions_count,
	.get_function_name = ingenic_pinmux_get_function_name,
	.get_function_groups = ingenic_pinmux_get_function_groups,
	.set_mux = ingenic_pinmux_set_mux,
	.gpio_set_direction = ingenic_pinmux_gpio_set_direction,
};

static int ingenic_pinconf_get(struct pinctrl_dev *pctldev,
		unsigned int pin, unsigned long *config)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	struct ingenic_gpio_chip *jzgc;
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned int idx, pull;

	if (pin >= (jzpc->num_gpio_chips * PINS_PER_GPIO_PORT))
		return -EINVAL;
	jzgc = &jzpc->gpio_chips[pin / PINS_PER_GPIO_PORT];
	idx = pin % PINS_PER_GPIO_PORT;

	pull = jzgc->ops->get_bias(jzgc->base, idx);

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

static int ingenic_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
		unsigned long *configs, unsigned int num_configs)
{
	struct ingenic_pinctrl *jzpc = pinctrl_dev_get_drvdata(pctldev);
	struct ingenic_gpio_chip *jzgc;
	unsigned int idx, cfg;

	if (pin >= (jzpc->num_gpio_chips * PINS_PER_GPIO_PORT))
		return -EINVAL;

	jzgc = &jzpc->gpio_chips[pin / PINS_PER_GPIO_PORT];
	idx = pin % PINS_PER_GPIO_PORT;

	for (cfg = 0; cfg < num_configs; cfg++) {
		switch (pinconf_to_config_param(configs[cfg])) {
		case PIN_CONFIG_BIAS_DISABLE:
			jzgc->ops->set_bias(jzgc->base, idx, false);
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
			if (!(jzgc->pull_ups & (1 << idx)))
				return -EINVAL;
			jzgc->ops->set_bias(jzgc->base, idx, true);
			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			if (!(jzgc->pull_downs & (1 << idx)))
				return -EINVAL;
			jzgc->ops->set_bias(jzgc->base, idx, true);
			break;
		default:
			dev_warn(jzpc->dev, "Invalid config for pin %s\n",
					jzpc->pdesc[pin].name);
			continue;
		}
	}

	return 0;
}

static struct pinconf_ops ingenic_confops = {
	.is_generic = true,
	.pin_config_get = ingenic_pinconf_get,
	.pin_config_set = ingenic_pinconf_set,
};

static int ingenic_pinctrl_parse_dt_gpio(struct ingenic_pinctrl *jzpc,
		struct ingenic_gpio_chip *jzgc, struct device_node *np)
{
	int err;

	jzgc->pinctrl = jzpc;
	snprintf(jzgc->name, sizeof(jzgc->name), "P%c", 'A' + jzgc->idx);

	jzgc->base = of_iomap(np, 0);
	if (!jzgc->base) {
		dev_err(jzpc->dev, "failed to map IO memory\n");
		return -ENXIO;
	}

	jzgc->gc.base = jzpc->base + (jzgc->idx * PINS_PER_GPIO_PORT);
	jzgc->gc.ngpio = PINS_PER_GPIO_PORT;
	jzgc->gc.parent = jzpc->dev;
	jzgc->gc.of_node = np;
	jzgc->gc.label = np->name;
	jzgc->gc.owner = THIS_MODULE;

	jzgc->gc.set = ingenic_gpio_set;
	jzgc->gc.get = ingenic_gpio_get;
	jzgc->gc.direction_input = ingenic_gpio_direction_input;
	jzgc->gc.direction_output = ingenic_gpio_direction_output;

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

	err = devm_gpiochip_add_data(jzpc->dev, &jzgc->gc, NULL);
	if (err)
		return err;

	if (!of_find_property(np, "interrupt-controller", NULL))
		return 0;

	jzgc->irq = irq_of_parse_and_map(np, 0);
	if (!jzgc->irq)
		return -EINVAL;

	jzgc->irq_chip.name = jzgc->name;
	jzgc->irq_chip.irq_unmask = ingenic_gpio_irq_unmask;
	jzgc->irq_chip.irq_mask = ingenic_gpio_irq_mask;
	jzgc->irq_chip.irq_ack = ingenic_gpio_irq_ack;
	jzgc->irq_chip.irq_set_type = ingenic_gpio_irq_set_type;

	err = gpiochip_irqchip_add(&jzgc->gc, &jzgc->irq_chip, 0,
			handle_level_irq, IRQ_TYPE_NONE);
	if (err)
		return err;

	gpiochip_set_chained_irqchip(&jzgc->gc, &jzgc->irq_chip,
			jzgc->irq, ingenic_gpio_irq_handler);
	return 0;
}

static int find_gpio_chip_by_of_node(struct gpio_chip *chip, void *data)
{
	return chip->of_node == data;
}

static int ingenic_pinctrl_parse_dt_pincfg(struct ingenic_pinctrl *jzpc,
		struct ingenic_pinctrl_pin *pin, phandle cfg_handle)
{
	struct device_node *cfg_node;
	int err;

	cfg_node = of_find_node_by_phandle(cfg_handle);
	if (!cfg_node)
		return -EINVAL;

	err = pinconf_generic_parse_dt_config(cfg_node, NULL,
			&pin->configs, &pin->num_configs);
	if (err)
		return err;

	err = devm_add_action(jzpc->dev, (void (*)(void *))kfree, pin->configs);
	if (err) {
		kfree(pin->configs);
		return err;
	}

	return 0;
}

static int ingenic_pinctrl_parse_dt_func(struct ingenic_pinctrl *jzpc,
		struct device_node *np, unsigned int *ifunc,
		unsigned int *igroup)
{
	struct ingenic_pinctrl_func *func;
	struct ingenic_pinctrl_group *grp;
	struct device_node *group_node, *gpio_node;
	struct gpio_chip *gpio_chip;
	phandle gpio_handle, cfg_handle;
	struct property *pp;
	__be32 *plist;
	unsigned int i, j;
	int err;
	const unsigned int vals_per_pin = 4;

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

			err = ingenic_pinctrl_parse_dt_pincfg(jzpc,
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

int ingenic_pinctrl_probe(struct platform_device *pdev,
		const struct ingenic_pinctrl_ops *ops)
{
	struct device *dev = &pdev->dev;
	struct ingenic_pinctrl *jzpc;
	struct ingenic_gpio_chip *jzgc;
	struct pinctrl_desc *pctl_desc;
	struct device_node *np, *chips_node, *functions_node;
	unsigned int i, j;
	int err;

	if (!dev->of_node) {
		dev_err(dev, "device tree node not found\n");
		return -ENODEV;
	}

	jzpc = devm_kzalloc(dev, sizeof(*jzpc), GFP_KERNEL);
	if (!jzpc)
		return -ENOMEM;

	jzpc->dev = dev;
	platform_set_drvdata(pdev, jzpc);

	jzpc->base = 0;
	of_property_read_u32(dev->of_node, "base", &jzpc->base);

	chips_node = of_find_node_by_name(dev->of_node, "gpio-chips");
	if (!chips_node) {
		dev_err(dev, "Missing \"chips\" devicetree node\n");
		return -EINVAL;
	}

	jzpc->num_gpio_chips = of_get_available_child_count(chips_node);
	if (!jzpc->num_gpio_chips) {
		dev_err(dev, "No GPIO chips found\n");
		return -EINVAL;
	}

	functions_node = of_find_node_by_name(dev->of_node, "functions");
	if (!functions_node) {
		dev_err(dev, "Missing \"functions\" devicetree node\n");
		return -EINVAL;
	}

	jzpc->num_funcs = of_get_available_child_count(functions_node);
	if (!jzpc->num_funcs) {
		dev_err(dev, "No functions found\n");
		return -EINVAL;
	}

	for_each_child_of_node(functions_node, np) {
		jzpc->num_groups += of_get_available_child_count(np);
	}

	if (!jzpc->num_groups) {
		dev_err(dev, "No groups found\n");
		return -EINVAL;
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

	/* fill in pinctrl_desc structure */
	pctl_desc->name = dev_name(dev);
	pctl_desc->owner = THIS_MODULE;
	pctl_desc->pctlops = &ingenic_pctlops;
	pctl_desc->pmxops = &ingenic_pmxops;
	pctl_desc->confops = &ingenic_confops;
	pctl_desc->npins = jzpc->num_gpio_chips * PINS_PER_GPIO_PORT;
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

	/* Register GPIO chips */

	i = 0;
	for_each_child_of_node(chips_node, np) {
		if (!of_find_property(np, "gpio-controller", NULL)) {
			dev_err(dev, "GPIO chip missing \"gpio-controller\" flag\n");
			return -EINVAL;
		}

		jzpc->gpio_chips[i].idx = i;
		jzpc->gpio_chips[i].ops = ops;

		err = ingenic_pinctrl_parse_dt_gpio(jzpc,
				&jzpc->gpio_chips[i++], np);
		if (err) {
			dev_err(dev, "failed to register GPIO chip: %d\n", err);
			return err;
		}
	}

	i = 0;
	for_each_child_of_node(functions_node, np) {
		err = ingenic_pinctrl_parse_dt_func(jzpc, np, &i, &j);
		if (err) {
			dev_err(dev, "failed to parse function %s\n",
					np->full_name);
			return err;
		}
	}

	for (i = 0; i < jzpc->num_groups; i++)
		dev_dbg(dev, "group '%s'\n", jzpc->groups[i].name);
	for (i = 0; i < jzpc->num_funcs; i++)
		dev_dbg(dev, "func '%s'\n", jzpc->funcs[i].name);

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
EXPORT_SYMBOL_GPL(ingenic_pinctrl_probe);

MODULE_AUTHOR("Paul Burton <paul.burton@imgtec.com>");
MODULE_DESCRIPTION("Ingenic pinctrl driver");
MODULE_LICENSE("GPL v2");

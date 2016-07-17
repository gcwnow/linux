#include "ingenic.h"

#include <asm/io.h>
#include <dt-bindings/interrupt-controller/irq.h>
#include <linux/errno.h>

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

static void jz4780_set_gpio(void __iomem *base,
		unsigned offset, bool output)
{
	writel(1 << offset, base + GPIO_INTC);
	writel(1 << offset, base + GPIO_MSKS);

	if (output)
		writel(1 << offset, base + GPIO_PAT1C);
	else
		writel(1 << offset, base + GPIO_PAT1S);
}

static int jz4780_get_bias(void __iomem *base, unsigned offset)
{
	return !((readl(base + GPIO_PEN) >> offset) & 0x1);
}

static void jz4780_set_bias(void __iomem *base, unsigned offset, bool enable)
{
	if (enable)
		writel(1 << offset, base + GPIO_PENC);
	else
		writel(1 << offset, base + GPIO_PENS);
}

static void jz4780_gpio_set_value(void __iomem *base,
		unsigned offset, int value)
{
	if (value)
		writel(1 << offset, base + GPIO_PAT0S);
	else
		writel(1 << offset, base + GPIO_PAT0C);
}

static int jz4780_gpio_get_value(void __iomem *base, unsigned offset)
{
	return readl(base + GPIO_PIN);
}

static u32 jz4780_irq_read(void __iomem *base)
{
	return readl(base + GPIO_FLG);
}

static void jz4780_irq_mask(void __iomem *base, unsigned offset, bool mask)
{
	if (mask)
		writel(1 << offset, base + GPIO_MSKS);
	else
		writel(1 << offset, base + GPIO_MSKC);
}

static void jz4780_irq_ack(void __iomem *base, unsigned offset)
{
	writel(1 << offset, base + GPIO_FLGC);
}

static void jz4780_irq_set_type(void __iomem *base,
		unsigned offset, unsigned type)
{
	enum {
		PAT_EDGE_RISING		= 0x3,
		PAT_EDGE_FALLING	= 0x2,
		PAT_LEVEL_HIGH		= 0x1,
		PAT_LEVEL_LOW		= 0x0,
	} pat;

	switch (type) {
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
	default:
		pat = PAT_LEVEL_LOW;
		break;
	};

	writel(1 << offset, base + ((pat & 0x2) ? GPIO_PAT1S : GPIO_PAT1C));
	writel(1 << offset, base + ((pat & 0x1) ? GPIO_PAT0S : GPIO_PAT0C));
	writel(1 << offset, base + GPIO_INTS);
}

static void jz4780_set_function(void __iomem *base,
		unsigned offset, unsigned func)
{
	writel(1 << offset, base + GPIO_INTC);
	writel(1 << offset, base + GPIO_MSKC);
	writel(1 << offset, base + ((func & 0x2) ? GPIO_PAT1S : GPIO_PAT1C));
	writel(1 << offset, base + ((func & 0x1) ? GPIO_PAT0S : GPIO_PAT0C));
}

struct ingenic_pinctrl_ops ingenic_pinctrl_ops = {
	.nb_functions	= 4,
	.set_function	= jz4780_set_function,
	.set_gpio	= jz4780_set_gpio,
	.set_bias	= jz4780_set_bias,
	.get_bias	= jz4780_get_bias,
	.gpio_set_value	= jz4780_gpio_set_value,
	.gpio_get_value	= jz4780_gpio_get_value,
	.irq_read	= jz4780_irq_read,
	.irq_mask	= jz4780_irq_mask,
	.irq_ack	= jz4780_irq_ack,
	.irq_set_type	= jz4780_irq_set_type,
};

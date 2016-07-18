#include "ingenic.h"

#include <asm/io.h>
#include <dt-bindings/interrupt-controller/irq.h>
#include <linux/errno.h>

/* GPIO port register offsets */
#define GPIO_PIN	0x00
#define GPIO_DATA	0x10
#define GPIO_DATAS	0x14
#define GPIO_DATAC	0x18
#define GPIO_MASK	0x20
#define GPIO_MASKS	0x24
#define GPIO_MASKC	0x28
#define GPIO_PULL_DIS	0x30
#define GPIO_PULL_DISS	0x34
#define GPIO_PULL_DISC	0x38
#define GPIO_FUNC	0x40
#define GPIO_FUNCS	0x44
#define GPIO_FUNCC	0x48
#define GPIO_SELECT	0x50
#define GPIO_SELECTS	0x54
#define GPIO_SELECTC	0x58
#define GPIO_DIR	0x60
#define GPIO_DIRS	0x64
#define GPIO_DIRC	0x68
#define GPIO_TRIG	0x70
#define GPIO_TRIGS	0x74
#define GPIO_TRIGC	0x78
#define GPIO_FLAG	0x80
#define GPIO_FLAGC	0x14
#define GPIO_REGS_SIZE	0x100

static void jz4740_set_gpio(void __iomem *base,
		unsigned offset, bool output)
{
	writel(1 << offset, base + GPIO_FUNCC);
	writel(1 << offset, base + GPIO_SELECTC);
	writel(1 << offset, base + GPIO_TRIGC);

	if (output)
		writel(1 << offset, base + GPIO_DIRS);
	else
		writel(1 << offset, base + GPIO_DIRC);
}

static int jz4740_get_bias(void __iomem *base, unsigned offset)
{
	return !((readl(base + GPIO_PULL_DIS) >> offset) & 0x1);
}

static void jz4740_set_bias(void __iomem *base, unsigned offset, bool enable)
{
	if (enable)
		writel(1 << offset, base + GPIO_PULL_DISC);
	else
		writel(1 << offset, base + GPIO_PULL_DISS);
}

static void jz4740_gpio_set_value(void __iomem *base,
		unsigned offset, int value)
{
	if (value)
		writel(1 << offset, base + GPIO_DATAS);
	else
		writel(1 << offset, base + GPIO_DATAC);
}

static int jz4740_gpio_get_value(void __iomem *base, unsigned offset)
{
	return (readl(base + GPIO_DATA) >> offset) & 0x1;
}

static u32 jz4740_irq_read(void __iomem *base)
{
	return readl(base + GPIO_FLAG);
}

static void jz4740_irq_mask(void __iomem *base, unsigned irq, bool mask)
{
	if (mask)
		writel(1 << irq, base + GPIO_MASKS);
	else
		writel(1 << irq, base + GPIO_MASKC);
}

static void jz4740_irq_ack(void __iomem *base, unsigned irq)
{
	writel(1 << irq, base + GPIO_FLAGC);
}

static void jz4740_irq_set_type(void __iomem *base,
		unsigned offset, unsigned type)
{
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		writel(1 << offset, base + GPIO_DIRS);
		writel(1 << offset, base + GPIO_TRIGS);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		writel(1 << offset, base + GPIO_DIRC);
		writel(1 << offset, base + GPIO_TRIGS);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		writel(1 << offset, base + GPIO_DIRS);
		writel(1 << offset, base + GPIO_TRIGC);
		break;
	case IRQ_TYPE_LEVEL_LOW:
	default:
		writel(1 << offset, base + GPIO_DIRC);
		writel(1 << offset, base + GPIO_TRIGC);
		break;
	}
}

static void jz4740_set_function(void __iomem *base,
		unsigned offset, unsigned func)
{
	writel(1 << offset, base + GPIO_FUNCS);
	writel(1 << offset, base + GPIO_TRIGC);

	switch (func) {
	case 2:
		writel(1 << offset, base + GPIO_TRIGS);
	case 1: /* fallthrough */
		writel(1 << offset, base + GPIO_SELECTS);
		break;
	case 0:
	default:
		writel(1 << offset, base + GPIO_SELECTC);
		break;
	}
}

#if 0
static void jz4740_irq_enable(void __iomem *base, unsigned irq, bool enable)
{
	writel(1 << irq, base + GPIO_SELECTS);
	writel(1 << irq, base + GPIO_MASKC);
}
#endif

struct ingenic_pinctrl_ops ingenic_pinctrl_ops = {
	.nb_functions	= 3,
	.set_function	= jz4740_set_function,
	.set_gpio	= jz4740_set_gpio,
	.set_bias	= jz4740_set_bias,
	.get_bias	= jz4740_get_bias,
	.gpio_set_value	= jz4740_gpio_set_value,
	.gpio_get_value	= jz4740_gpio_get_value,
//	.irq_enable	= jz4740_irq_enable,
	.irq_read	= jz4740_irq_read,
	.irq_mask	= jz4740_irq_mask,
	.irq_ack	= jz4740_irq_ack,
	.irq_set_type	= jz4740_irq_set_type,
};

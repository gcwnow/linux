/*
 * Author: River <zwang@ingenic.cn>
 * Restructured by Maarten ter Huurne <maarten@treewalker.org>, using the
 * tusb6010 module as a template.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/platform_data/usb-musb-jz4770.h>

#include <asm/mach-jz4770/gpio.h>

#include "musb_core.h"

#define REG_USBPCR_OFFSET	0x00
#define REG_USBRDT_OFFSET	0x04
#define REG_USBVBFIL_OFFSET	0x08
#define REG_USBPCR1_OFFSET	0x0c

/* USBPCR */
#define USBPCR_USB_MODE		BIT(31)
#define USBPCR_AVLD_REG		BIT(30)
#define USBPCR_INCRM		BIT(27)	/* INCR_MASK bit */
#define USBPCR_CLK12_EN		BIT(26)
#define USBPCR_COMMONONN	BIT(25)
#define USBPCR_VBUSVLDEXT	BIT(24)
#define USBPCR_VBUSVLDEXTSEL	BIT(23)
#define USBPCR_POR		BIT(22)
#define USBPCR_SIDDQ		BIT(21)
#define USBPCR_OTG_DISABLE	BIT(20)
#define USBPCR_TXPREEMPHTUNE	BIT(6)

#define USBPCR_IDPULLUP_LSB	28	/* IDPULLUP_MASK bit */
#define USBPCR_IDPULLUP_MASK	GENMASK(29, USBPCR_IDPULLUP_LSB)
#define USBPCR_IDPULLUP_ALWAYS	(3 << USBPCR_IDPULLUP_LSB)
#define USBPCR_IDPULLUP_SUSPEND	(1 << USBPCR_IDPULLUP_LSB)
#define USBPCR_IDPULLUP_OTG	(0 << USBPCR_IDPULLUP_LSB)

#define USBPCR_COMPDISTUNE_LSB	17
#define USBPCR_COMPDISTUNE_MASK	GENMASK(19, USBPCR_COMPDISTUNE_LSB)

#define USBPCR_OTGTUNE_LSB	14
#define USBPCR_OTGTUNE_MASK	GENMASK(16, USBPCR_OTGTUNE_LSB)

#define USBPCR_SQRXTUNE_LSB	11
#define USBPCR_SQRXTUNE_MASK	GENMASK(13, USBPCR_SQRXTUNE_LSB)

#define USBPCR_TXFSLSTUNE_LSB	7
#define USBPCR_TXFSLSTUNE_MASK	GENMASK(10, USBPCR_TXFSLSTUNE_LSB)

#define USBPCR_TXRISETUNE_LSB	4
#define USBPCR_TXRISETUNE_MASK	GENMASK(5, USBPCR_TXRISETUNE_LSB)

#define USBPCR_TXVREFTUNE_LSB	0
#define USBPCR_TXVREFTUNE_MASK	GENMASK(3, USBPCR_TXVREFTUNE_LSB)

/* USBRDT */
#define USBRDT_VBFIL_LD_EN	BIT(25)
#define USBRDT_IDDIG_EN		BIT(24)
#define USBRDT_IDDIG_REG	BIT(23)

#define USBRDT_USBRDT_LSB	0
#define USBRDT_USBRDT_MASK	GENMASK(22, USBRDT_USBRDT_LSB)

/* USBPCR1 */
#define USBPCR1_UHC_POWON	BIT(5)

struct jz_musb_glue {
	void __iomem *base;
	struct device *dev;
	struct platform_device *musb;
	struct clk *clk;
	struct timer_list gpio_id_debounce_timer;
	unsigned long gpio_id_debounce_jiffies;
	bool enabled;
};

static inline void jz_musb_phy_reset(struct jz_musb_glue *glue)
{
	u32 reg = readl(glue->base + REG_USBPCR_OFFSET);

	writel(reg | USBPCR_POR, glue->base + REG_USBPCR_OFFSET);
	udelay(30);
	writel(reg & ~USBPCR_POR, glue->base + REG_USBPCR_OFFSET);

	udelay(300);
}

static inline void jz_musb_set_device_only_mode(struct jz_musb_glue *glue)
{
	u32 reg = readl(glue->base + REG_USBPCR_OFFSET);

	dev_info(glue->dev, "Device only mode.\n");

	/* Device Mode. */
	reg = (reg | USBPCR_VBUSVLDEXT) & ~USBPCR_USB_MODE;
	writel(reg, glue->base + REG_USBPCR_OFFSET);
}

static inline void jz_musb_set_normal_mode(struct jz_musb_glue *glue)
{
	u32 reg = readl(glue->base + REG_USBPCR_OFFSET);

	dev_info(glue->dev, "Normal mode.\n");

	reg = (reg & ~(
			USBPCR_VBUSVLDEXT |
			USBPCR_VBUSVLDEXTSEL |
			USBPCR_OTG_DISABLE |
			USBPCR_IDPULLUP_MASK)) |
		USBPCR_USB_MODE | USBPCR_IDPULLUP_ALWAYS;

	writel(reg, glue->base + REG_USBPCR_OFFSET);
}

static inline void jz_musb_init_regs(struct jz_musb_glue *glue)
{
	u32 reg;

	/* fil */
	writel(0x80, glue->base + REG_USBVBFIL_OFFSET);

	/* rdt */
	writel(0x02000096, glue->base + REG_USBRDT_OFFSET);

	/* TXRISETUNE & TXVREFTUNE. */
	reg = readl(glue->base + REG_USBPCR_OFFSET);
	reg = (reg & ~(USBPCR_TXRISETUNE_MASK | USBPCR_TXVREFTUNE_MASK)) |
		(3 << USBPCR_TXRISETUNE_LSB) |
		(5 << USBPCR_TXVREFTUNE_LSB);
	writel(reg, glue->base + REG_USBPCR_OFFSET);

	jz_musb_set_normal_mode(glue);
	jz_musb_phy_reset(glue);
}

static void jz_musb_set_vbus(struct musb *musb, int is_on)
{
	u8 devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv->otg->default_a = 1;
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);

		//act8600_set_power_mode(VBUS_POWERED_BY_5VIN);
	} else {
		musb->is_active = 0;

		//act8600_set_power_mode(VBUS_POWERED_EXTERNALLY);

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv->otg->default_a = 0;
		musb->xceiv->otg->state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	dev_dbg(musb->xceiv->dev, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		usb_otg_state_string(musb->xceiv->otg->state),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

/* ---------------------- OTG ID PIN Routines ---------------------------- */

static void do_otg_id_pin_state(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *pdata = dev->platform_data;
	struct jz_otg_board_data *board_data = pdata->board_data;

	unsigned int default_a = !gpio_get_value(board_data->gpio_id_pin);

	dev_info(dev->parent, "USB OTG default mode: %s\n",
			default_a ? "A" : "B");

	musb->xceiv->otg->default_a = default_a;

	jz_musb_set_vbus(musb, default_a);
}

static void otg_id_pin_stable_func(unsigned long data)
{
	struct musb *musb = (struct musb *)data;

	do_otg_id_pin_state(musb);
}

static irqreturn_t jz_musb_otg_id_irq(int irq, void *data)
{
	struct jz_musb_glue *glue = data;

	mod_timer(&glue->gpio_id_debounce_timer,
		  jiffies + glue->gpio_id_debounce_jiffies);

	return IRQ_HANDLED;
}

static int otg_id_pin_setup(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct jz_musb_glue *glue = dev_get_drvdata(dev->parent);
	struct musb_hdrc_platform_data *pdata = dev->platform_data;
	struct jz_otg_board_data *board_data = pdata->board_data;
	int id_pin = board_data->gpio_id_pin;
	int ret;

	ret = devm_gpio_request(dev, id_pin, "USB OTG ID");
	if (ret) {
		dev_err(dev, "Failed to request USB OTG ID pin %d: %d\n",
			id_pin, ret);
		return ret;
	}

	gpio_direction_input(id_pin);

	glue->gpio_id_debounce_jiffies =
			msecs_to_jiffies(board_data->gpio_id_debounce_ms);

	/* Update OTG ID PIN state. */
	do_otg_id_pin_state(musb);
	setup_timer(&glue->gpio_id_debounce_timer, otg_id_pin_stable_func,
		    (unsigned long)musb);

	ret = devm_request_irq(dev, gpio_to_irq(id_pin), jz_musb_otg_id_irq,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "otg-id-irq", glue);
	if (ret) {
		dev_err(dev, "Failed to request USB OTG ID IRQ: %d\n", ret);
		return ret;
	}

	return ret;
}

static void otg_id_pin_cleanup(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct jz_musb_glue *glue = dev_get_drvdata(dev->parent);
	struct musb_hdrc_platform_data *pdata = dev->platform_data;
	struct jz_otg_board_data *board_data = pdata->board_data;

	devm_free_irq(dev, gpio_to_irq(board_data->gpio_id_pin), glue);
	del_timer(&glue->gpio_id_debounce_timer);
}

/* ---------------------------------------------------------------- */

static irqreturn_t jz_musb_interrupt(int irq, void *__hci)
{
	unsigned long	flags;
	struct musb	*musb = __hci;

	irqreturn_t rv, rv_dma, rv_usb;
	rv = rv_dma = rv_usb = IRQ_NONE;

	spin_lock_irqsave(&musb->lock, flags);

#if defined(CONFIG_USB_INVENTRA_DMA)
	if (musb->b_dma_share_usb_irq)
		rv_dma = musb_call_dma_controller_irq(irq, musb);
#endif

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		rv_usb = musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	rv = (rv_dma == IRQ_HANDLED || rv_usb == IRQ_HANDLED) ?
		IRQ_HANDLED : IRQ_NONE;

	return rv;
}

static void jz_musb_platform_enable(struct musb *musb)
{
	struct device *dev = musb->controller->parent;
	struct jz_musb_glue *glue = dev_get_drvdata(dev);

	if (!glue->enabled) {
		dev_info(dev, "Enable USB PHY.\n");
		usb_phy_init(musb->xceiv);

		glue->enabled = true;
	}
}

static void jz_musb_platform_disable(struct musb *musb)
{
	struct device *dev = musb->controller->parent;
	struct jz_musb_glue *glue = dev_get_drvdata(dev);

	if (glue->enabled) {
		dev_info(dev, "Disable USB PHY.\n");
		usb_phy_shutdown(musb->xceiv);

		glue->enabled = false;
	}
}

static int jz_musb_platform_init(struct musb *musb)
{
	struct device *dev = musb->controller->parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct jz_musb_glue *glue = dev_get_drvdata(dev);
	struct resource *mem;
	struct clk *clk;

	musb->xceiv = devm_usb_get_phy_by_phandle(dev, "usb-phy", 0);
	if (IS_ERR(musb->xceiv))
		musb->xceiv = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (IS_ERR(musb->xceiv)) {
		int ret = PTR_ERR(musb->xceiv);
		dev_err(dev, "Failed to get PHY: %d\n", ret);
		return ret;
	}

	musb->b_dma_share_usb_irq = 1;
	musb->isr = jz_musb_interrupt;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	glue->base = devm_ioremap_resource(dev, mem);
	if (IS_ERR(glue->base)) {
		int ret = PTR_ERR(glue->base);
		dev_err(dev, "Failed to map registers: %d\n", ret);
		return ret;
	}

	clk = devm_clk_get(dev, "usb");
	if (IS_ERR(clk)) {
		int ret = PTR_ERR(clk);
		dev_err(dev, "Failed to get clock: %d\n", ret);
		return ret;
	}

	glue->clk = clk;

	clk_prepare_enable(glue->clk);
	jz_musb_init_regs(glue);

	/* host mode and otg(host) depend on the id pin */
	return otg_id_pin_setup(musb);
}

static int jz_musb_platform_exit(struct musb *musb)
{
	struct jz_musb_glue *glue = dev_get_drvdata(musb->controller->parent);

	jz_musb_platform_disable(musb);

	clk_disable_unprepare(glue->clk);

	otg_id_pin_cleanup(musb);

	return 0;
}

static const struct musb_platform_ops jz_musb_ops = {
	.quirks		= MUSB_DMA_INVENTRA | MUSB_INDEXED_EP,
	.fifo_mode	= 2,

	.init		= jz_musb_platform_init,
	.exit		= jz_musb_platform_exit,

#ifdef CONFIG_USB_INVENTRA_DMA
	.dma_init	= musbhs_dma_controller_create,
	.dma_exit	= musbhs_dma_controller_destroy,
#endif

	.enable		= jz_musb_platform_enable,
	.disable	= jz_musb_platform_disable,

	.set_vbus	= jz_musb_set_vbus,
};

static struct musb_hdrc_config jz_musb_config = {
	.multipoint	= 1,
/* Max EPs scanned. Driver will decide which EP can be used automatically. */
	.num_eps	= 6,
};

static struct jz_otg_board_data gcw0_otg_board_data = {
	.gpio_id_pin = JZ_GPIO_PORTF(18),
	.gpio_id_debounce_ms = 500,
};

static int jz_musb_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata = pdev->dev.platform_data;
	struct platform_device		*musb;
	struct jz_musb_glue		*glue;
	struct device_node		*node = pdev->dev.of_node;

	int				ret = -ENOMEM;

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		goto err0;

	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			goto err0;
		/* TODO: Read the board data from the device tree.
		 *       Alternatively, use the ID pin support from the generic
		 *       PHY driver and remove glue code using the board data.
		 */
		pdata->board_data = &gcw0_otg_board_data;
	}

	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err0;
	}

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= &musb->dev.coherent_dma_mask;
	musb->dev.coherent_dma_mask	= DMA_BIT_MASK(32);

	glue->dev			= &pdev->dev;
	glue->musb			= musb;

	pdata->platform_ops		= &jz_musb_ops;
	if (!pdata->config)
		pdata->config = &jz_musb_config;

	if (node) {
		u32 mode;
		if (!of_property_read_u32(node, "mode", &mode))
			pdata->mode = mode;
	}

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
			pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err1;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err1;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err1;
	}

	return 0;

err1:
	platform_device_put(musb);

err0:
	return ret;
}

static int jz_musb_remove(struct platform_device *pdev)
{
	struct jz_musb_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);

	return 0;
}

static const struct of_device_id jz_musb_of_match[] = {
	{ .compatible = "ingenic,jz4770-musb", },
	{ },
};
MODULE_DEVICE_TABLE(of, jz_musb_of_match);

static struct platform_driver jz_musb_driver = {
	.probe		= jz_musb_probe,
	.remove		= jz_musb_remove,
	.driver		= {
		.name	= "musb-jz",
		.of_match_table = jz_musb_of_match,
	},
};

module_platform_driver(jz_musb_driver);

MODULE_DESCRIPTION("JZ4770 MUSB Glue Layer");
MODULE_AUTHOR("River <zwang@ingenic.cn>");
MODULE_LICENSE("GPL v2");

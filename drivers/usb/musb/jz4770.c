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
#include <linux/usb/otg.h>
#include <linux/usb/nop-usb-xceiv.h>

#include <asm/mach-jz4770/board-gcw0.h>
#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770gpio.h>

#include "musb_core.h"


struct jz_musb_glue {
	struct device		*dev;
	struct platform_device	*musb;
};

static inline void jz_musb_phy_enable(void)
{
	printk(KERN_INFO "jz4760: Enable USB PHY.\n");

	__cpm_enable_otg_phy();

	/* Wait PHY Clock Stable. */
	udelay(300);
}

static inline void jz_musb_phy_disable(void)
{
	printk(KERN_INFO "jz4760: Disable USB PHY.\n");

	__cpm_suspend_otg_phy();
}

static inline void jz_musb_phy_reset(void)
{
	REG_CPM_USBPCR |= USBPCR_POR;
	udelay(30);
	REG_CPM_USBPCR &= ~USBPCR_POR;

	udelay(300);
}

static inline void jz_musb_set_device_only_mode(void)
{
	printk(KERN_INFO "jz4760: Device only mode.\n");

	/* Device Mode. */
	REG_CPM_USBPCR &= ~(1 << 31);

	REG_CPM_USBPCR |= USBPCR_VBUSVLDEXT;
}

static inline void jz_musb_set_normal_mode(void)
{
	printk(KERN_INFO "jz4760: Normal mode.\n");

	__gpio_as_otg_drvvbus();

	/* OTG Mode. */
	REG_CPM_USBPCR |= (1 << 31);

	REG_CPM_USBPCR &= ~((1 << 24) | (1 << 23) | (1 << 20));

	REG_CPM_USBPCR |= ((1 << 28) | (1 << 29));
}

static inline void jz_musb_init_regs(struct musb *musb)
{
	/* fil */
	REG_CPM_USBVBFIL = 0x80;

	/* rdt */
	REG_CPM_USBRDT = 0x96;

	/* rdt - filload_en */
	REG_CPM_USBRDT |= (1 << 25);

	/* TXRISETUNE & TXVREFTUNE. */
	REG_CPM_USBPCR &= ~0x3f;
	REG_CPM_USBPCR |= 0x35;

	jz_musb_set_normal_mode();

	jz_musb_phy_reset();
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
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv->otg->default_a = 0;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	dev_dbg(musb->xceiv->dev, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb->xceiv->state),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

/* ---------------------- OTG ID PIN Routines ---------------------------- */

static struct timer_list otg_id_pin_stable_timer;

static unsigned int read_gpio_pin(unsigned int pin, unsigned int loop)
{
	unsigned int t, v;
	unsigned int i;

	i = loop;

	v = t = 0;

	while (i--) {
		t = __gpio_get_pin(pin);
		if (v != t)
			i = loop;

		v = t;
	}

	return v;
}

static void do_otg_id_pin_state(struct musb *musb)
{
	unsigned int default_a;
	unsigned int pin = read_gpio_pin(GPIO_OTG_ID_PIN, 5000);

	default_a = !pin;

	musb->xceiv->otg->default_a = default_a;

	jz_musb_set_vbus(musb, default_a);

	if (pin) {
		/* B */
#ifdef CONFIG_USB_MUSB_PERIPHERAL_HOTPLUG
		__gpio_unmask_irq(OTG_HOTPLUG_PIN);
#endif
		__gpio_as_irq_fall_edge(GPIO_OTG_ID_PIN);
	} else {
		/* A */
#ifdef CONFIG_USB_MUSB_PERIPHERAL_HOTPLUG
		__gpio_mask_irq(OTG_HOTPLUG_PIN); // otg's host mode not support hotplug
#endif
		__gpio_as_irq_rise_edge(GPIO_OTG_ID_PIN);
	}
}

static void otg_id_pin_stable_func(unsigned long data)
{
	struct musb *musb = (struct musb *)data;

	do_otg_id_pin_state(musb);
}

static irqreturn_t jz_musb_otg_id_irq(int irq, void *data)
{
	mod_timer(&otg_id_pin_stable_timer, GPIO_OTG_STABLE_JIFFIES + jiffies);

	return IRQ_HANDLED;
}

static int otg_id_pin_setup(struct musb *musb)
{
	int rv;

	/* Update OTG ID PIN state. */
	do_otg_id_pin_state(musb);
	setup_timer(&otg_id_pin_stable_timer, otg_id_pin_stable_func, (unsigned long)musb);

	rv = request_irq(GPIO_OTG_ID_IRQ, jz_musb_otg_id_irq,
				IRQF_DISABLED, "otg-id-irq", musb);
	if (rv) {
		pr_err("Failed to request OTG_ID_IRQ.\n");
		return rv;
	}

	return rv;
}

static void otg_id_pin_cleanup(struct musb *musb)
{
	free_irq(GPIO_OTG_ID_IRQ, "otg-id-irq");
	del_timer(&otg_id_pin_stable_timer);
}

/* ---------------------------------------------------------------- */

static int __init jz_musb_platform_init(struct musb *musb)
{
	musb->xceiv = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!musb->xceiv) {
		pr_err("HS USB OTG: no transceiver configured\n");
		return -ENODEV;
	}

	musb->b_dma_share_usb_irq = 1;

	jz_musb_init_regs(musb);

	/* host mode and otg(host) depend on the id pin */
	return otg_id_pin_setup(musb);
}

static int jz_musb_platform_exit(struct musb *musb)
{
	jz_musb_phy_disable();

	otg_id_pin_cleanup(musb);

	usb_nop_xceiv_unregister();

	return 0;
}

static void jz_musb_platform_enable(struct musb *musb)
{
	jz_musb_phy_enable();
}

static void jz_musb_platform_disable(struct musb *musb)
{
	jz_musb_phy_disable();
}

static const struct musb_platform_ops jz_musb_ops = {
	.init		= jz_musb_platform_init,
	.exit		= jz_musb_platform_exit,

	.enable		= jz_musb_platform_enable,
	.disable	= jz_musb_platform_disable,

	.set_vbus	= jz_musb_set_vbus,
};

static u64 jz_musb_dmamask = DMA_BIT_MASK(32);

static int __init jz_musb_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata = pdev->dev.platform_data;
	struct platform_device		*musb;
	struct jz_musb_glue		*glue;

	int				ret = -ENOMEM;

	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "failed to allocate glue context\n");
		goto err0;
	}

	musb = platform_device_alloc("musb-hdrc", -1);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err1;
	}

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= &jz_musb_dmamask;
	musb->dev.coherent_dma_mask	= jz_musb_dmamask;

	glue->dev			= &pdev->dev;
	glue->musb			= musb;

	pdata->platform_ops		= &jz_musb_ops;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
			pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err2;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err2;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err2;
	}

	return 0;

err2:
	platform_device_put(musb);

err1:
	kfree(glue);

err0:
	return ret;
}

static int __exit jz_musb_remove(struct platform_device *pdev)
{
	struct jz_musb_glue		*glue = platform_get_drvdata(pdev);

	platform_device_del(glue->musb);
	platform_device_put(glue->musb);
	kfree(glue);

	return 0;
}

static struct platform_driver jz_musb_driver = {
	.remove		= __exit_p(jz_musb_remove),
	.driver		= {
		.name	= "musb-jz",
	},
};

static int __init jz_musb_init(void)
{
	return platform_driver_probe(&jz_musb_driver, jz_musb_probe);
}
subsys_initcall(jz_musb_init);

static void __exit jz_musb_exit(void)
{
	platform_driver_unregister(&jz_musb_driver);
}
module_exit(jz_musb_exit);

MODULE_DESCRIPTION("JZ4770 MUSB Glue Layer");
MODULE_AUTHOR("River <zwang@ingenic.cn>");
MODULE_LICENSE("GPL v2");

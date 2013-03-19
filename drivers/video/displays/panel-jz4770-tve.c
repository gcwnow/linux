/*
 * panel-jz4770-tve.c -- Ingenic JZ4770 TV encoder
 *
 * Copyright (C) 2013, Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <video/jzpanel.h>
#include <video/panel-jz4770-tve.h>


#define JZ_REG_TVE_CTRL		0x00	/* control */
#define JZ_REG_TVE_FRCFG	0x04	/* frame */
#define JZ_REG_TVE_SLCFG1	0x10	/* signal level */
#define JZ_REG_TVE_SLCFG2	0x14
#define JZ_REG_TVE_SLCFG3	0x18
#define JZ_REG_TVE_LTCFG1	0x20	/* line timing */
#define JZ_REG_TVE_LTCFG2	0x24
#define JZ_REG_TVE_CFREQ	0x30	/* chroma sub-carrier freq */
#define JZ_REG_TVE_CPHASE	0x34	/* chroma sub-carrier phase */
#define JZ_REG_TVE_CCFG		0x38	/* chroma filter */
#define JZ_REG_TVE_WSSCR	0x40	/* wide screen signal */
#define JZ_REG_TVE_WSSCFG1	0x44
#define JZ_REG_TVE_WSSCFG2	0x48
#define JZ_REG_TVE_WSSCFG3	0x4C


struct jz4770_tve {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	unsigned int tv_norm;
};

static struct jz4770_tve jz4770_tve_panel;

#define JZ4770_TVE_NORM_NTSC	0
#define JZ4770_TVE_NORM_PAL50	1

static const char *jz4770_tve_norms[] = {
	"ntsc", "pal",
};

static int jz4770_tve_norm_set(struct jz4770_tve *panel, unsigned int norm)
{
	printk("GCW0 TV out: %d\n", norm);
	if (norm >= ARRAY_SIZE(jz4770_tve_norms))
		return -EINVAL;
	if (norm == panel->tv_norm)
		return 0;
	printk("Switch to %s\n", jz4770_tve_norms[norm]);

	if (true) {
		printk("Norm switching not implemented yet!\n");
		return -ENOSYS;
	}

	panel->tv_norm = norm;
	return 0;
}

static ssize_t jz4770_tve_norm_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct jz4770_tve *panel = &jz4770_tve_panel;

	if (panel->tv_norm >= ARRAY_SIZE(jz4770_tve_norms)) {
		dev_err(dev, "Unknown norm for TV-out\n");
		return -1;
	}

	return sprintf(buf, "%s\n", jz4770_tve_norms[panel->tv_norm]);
}

static ssize_t jz4770_tve_norm_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t n)
{
	struct jz4770_tve *panel = &jz4770_tve_panel;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(jz4770_tve_norms); i++) {
		if (sysfs_streq(jz4770_tve_norms[i], buf)) {
			jz4770_tve_norm_set(panel, i);
			return n;
		}
	}
	return -EINVAL;
}

static DEVICE_ATTR(tv_norm, 0644, jz4770_tve_norm_show, jz4770_tve_norm_store);

static int __devinit jz4770_tve_panel_init(void **out_panel,
					   struct device *dev,
					   void *panel_pdata)
{
	struct jz4770_tve *panel = &jz4770_tve_panel;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *mem;
	int ret;

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tve");
	if (!mem) {
		dev_err(&pdev->dev, "Failed to get resource for TVE regs\n");
		return -ENOENT;
	}

	panel->base = devm_request_and_ioremap(dev, mem);
	if (!panel->base) {
		dev_err(&pdev->dev, "Failed to ioremap TVE regs\n");
		return -EBUSY;
	}

	panel->clk = devm_clk_get(dev, "tve");
	if (IS_ERR(panel->clk)) {
		dev_err(&pdev->dev,
			"Failed to get TVE clock: %ld\n", PTR_ERR(panel->clk));
		return PTR_ERR(panel->clk);
	}

	ret = device_create_file(dev, &dev_attr_tv_norm);
	if (ret)
		return ret;

	panel->dev = dev;

	*out_panel = panel;
	return 0;
}

static void __devexit jz4770_tve_panel_exit(void *panel_ptr)
{
	struct jz4770_tve *panel = panel_ptr;

	device_remove_file(panel->dev, &dev_attr_tv_norm);
}

static void jz4770_tve_panel_enable(void *panel)
{
}

static void jz4770_tve_panel_disable(void *panel)
{
}

struct panel_ops jz4770_tve_panel_ops = {
	.init		= jz4770_tve_panel_init,
	.exit		= jz4770_tve_panel_exit,
	.enable		= jz4770_tve_panel_enable,
	.disable	= jz4770_tve_panel_disable,
};

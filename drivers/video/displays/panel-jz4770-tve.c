/*
 * panel-jz4770-tve.c -- Ingenic JZ4770 TV encoder
 *
 * Copyright (C) 2013, Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/io.h>

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <video/jzpanel.h>
#include <video/panel-jz4770-tve.h>


/* Registers */
#define JZ_REG_TVE_CTRL			0x00	/* control */
#define JZ_REG_TVE_FRCFG		0x04	/* frame */
#define JZ_REG_TVE_SLCFG1		0x10	/* signal level */
#define JZ_REG_TVE_SLCFG2		0x14
#define JZ_REG_TVE_SLCFG3		0x18
#define JZ_REG_TVE_LTCFG1		0x20	/* line timing */
#define JZ_REG_TVE_LTCFG2		0x24
#define JZ_REG_TVE_CFREQ		0x30	/* chroma sub-carrier freq */
#define JZ_REG_TVE_CPHASE		0x34	/* chroma sub-carrier phase */
#define JZ_REG_TVE_CCFG			0x38	/* chroma filter */
#define JZ_REG_TVE_WSSCR		0x40	/* wide screen signal */
#define JZ_REG_TVE_WSSCFG1		0x44
#define JZ_REG_TVE_WSSCFG2		0x48
#define JZ_REG_TVE_WSSCFG3		0x4C

/* Control register fields */
#define JZ_TVE_CTRL_DAPD1		BIT(21)	/* DAC 1 power down */
#define JZ_TVE_CTRL_DAPD		BIT(20)	/* all DAC power down */
#define JZ_TVE_CTRL_CLBAR		BIT(4)	/* color bar mode */
#define JZ_TVE_CTRL_SWRST		BIT(0)	/* software reset */

/* Frame register fields */
#define JZ_TVE_FRCFG_L1ST_BIT		16
#define JZ_TVE_FRCFG_NLINE_BIT		0

/* Line timing register fields */
#define JZ_TVE_LTCFG1_FRONTP_BIT	16
#define JZ_TVE_LTCFG1_HSYNCW_BIT	8
#define JZ_TVE_LTCFG1_BACKP_BIT		0
#define JZ_TVE_LTCFG2_ACTLIN_BIT	16
#define JZ_TVE_LTCFG2_PREBW_BIT		8
#define JZ_TVE_LTCFG2_BURSTW_BIT	0

/* Chroma register fields */
#define JZ_TVE_CPHASE_INITPH_BIT	24
#define JZ_TVE_CPHASE_ACTPH_BIT		16
#define JZ_TVE_CPHASE_CCRSTP_BIT	0
#define JZ_TVE_CCFG_CBBA_BIT		24
#define JZ_TVE_CCFG_CRBA_BIT		16
#define JZ_TVE_CCFG_CBGAIN_BIT		8
#define JZ_TVE_CCFG_CRGAIN_BIT		0

struct jz4770_tve {
	struct device *dev;
	void __iomem *base;
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

static void jz4770_tve_set_enabled(struct jz4770_tve *panel, bool enabled)
{
	u32 ctrl = readl(panel->base + JZ_REG_TVE_CTRL);
	if (enabled) {
		ctrl |= JZ_TVE_CTRL_DAPD1;
		writel(ctrl, panel->base + JZ_REG_TVE_CTRL);
		mdelay(1);
		//ctrl &= ~BIT(9) /* SYNCT for NTSC */;
		//ctrl &= ~BIT(8) /* NTSC */;
		/* Note: Contrary to what the documentation says, the hardware
		 *       will power down the DAC if DAPD is set.
		 */
		ctrl &= ~JZ_TVE_CTRL_DAPD;
		ctrl &= ~JZ_TVE_CTRL_SWRST;
	} else {
		ctrl &= ~JZ_TVE_CTRL_DAPD1;
		ctrl |= JZ_TVE_CTRL_SWRST;
	}
	writel(ctrl, panel->base + JZ_REG_TVE_CTRL);
	dev_info(panel->dev, "TVE CTRL = %08X\n", ctrl);
	// jz-lcd jz-lcd.0: TVE CTRL = 21340210
}

static void jz4770_tve_set_chroma_ntsc(struct jz4770_tve *panel)
{
	writel(0x21F07C1F, panel->base + JZ_REG_TVE_CFREQ);
	writel((0x17 << JZ_TVE_CPHASE_INITPH_BIT)
	     | (   0 << JZ_TVE_CPHASE_ACTPH_BIT)
	     | (   1 << JZ_TVE_CPHASE_CCRSTP_BIT),
	       panel->base + JZ_REG_TVE_CPHASE);
	writel((  59 << JZ_TVE_CCFG_CBBA_BIT)
	     | (   0 << JZ_TVE_CCFG_CRBA_BIT)
	     | ( 137 << JZ_TVE_CCFG_CBGAIN_BIT)
	     | ( 137 << JZ_TVE_CCFG_CRGAIN_BIT),
	       panel->base + JZ_REG_TVE_CCFG);
}

static void jz4770_tve_set_chroma_pal(struct jz4770_tve *panel)
{
	//writel(0x21F07C1F, panel->base + JZ_REG_TVE_CFREQ);
	writel(0x2A098ACB, panel->base + JZ_REG_TVE_CFREQ);
	writel((   0 << JZ_TVE_CPHASE_INITPH_BIT)
	     | (   0 << JZ_TVE_CPHASE_ACTPH_BIT)
	     | (   1 << JZ_TVE_CPHASE_CCRSTP_BIT),
	       panel->base + JZ_REG_TVE_CPHASE);
#if 1
	writel((  59 << JZ_TVE_CCFG_CBBA_BIT)
	     | (  59 << JZ_TVE_CCFG_CRBA_BIT)
	     | ( 137 << JZ_TVE_CCFG_CBGAIN_BIT)
	     | ( 137 << JZ_TVE_CCFG_CRGAIN_BIT),
	       panel->base + JZ_REG_TVE_CCFG);
#else
	writel((  23 << JZ_TVE_CCFG_CBBA_BIT)
	     | (  23 << JZ_TVE_CCFG_CRBA_BIT)
	     | ( 137 << JZ_TVE_CCFG_CBGAIN_BIT)
	     | ( 137 << JZ_TVE_CCFG_CRGAIN_BIT),
	       panel->base + JZ_REG_TVE_CCFG);
#endif
}

static void jz4770_tve_configure_test_bars(struct jz4770_tve *panel)
{
	u32 ctrl;

	ctrl = readl(panel->base + JZ_REG_TVE_CTRL);
	//ctrl |= BIT(24) /*TVE_CTRL_ECVBS*/;
	//ctrl |= 4 << 16 /*TVE_CTRL_YCDLY_BIT*/;
	ctrl |= JZ_TVE_CTRL_SWRST;
	//ctrl |= JZ_TVE_CTRL_CLBAR;
	if (1) { /* NTSC */
		ctrl &= ~BIT(9);
		ctrl &= ~BIT(8);
	} else { /* PAL */
		ctrl |= BIT(9);
		ctrl |= BIT(8);
	}
	writel(ctrl, panel->base + JZ_REG_TVE_CTRL);

	/* Frame timing */
	writel((  22 << JZ_TVE_FRCFG_L1ST_BIT) // was: 21
	     | ( 524 << JZ_TVE_FRCFG_NLINE_BIT), // was: 525
	       panel->base + JZ_REG_TVE_FRCFG);

	/* Line timing */
	writel((  16 << JZ_TVE_LTCFG1_FRONTP_BIT)
	     | (  63 << JZ_TVE_LTCFG1_HSYNCW_BIT)
	     | (  59 << JZ_TVE_LTCFG1_BACKP_BIT),
	       panel->base + JZ_REG_TVE_LTCFG1);
	writel((1408 << JZ_TVE_LTCFG2_ACTLIN_BIT)
	     | (  22 << JZ_TVE_LTCFG2_PREBW_BIT)
	     | (  68 << JZ_TVE_LTCFG2_BURSTW_BIT),
	       panel->base + JZ_REG_TVE_LTCFG2);

	//jz4770_tve_set_chroma_pal(panel);
	jz4770_tve_set_chroma_ntsc(panel);
}

static int jz4770_tve_panel_init(void **out_panel,
					   struct device *dev,
					   void *panel_pdata)
{
	struct jz4770_tve *panel = &jz4770_tve_panel;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *mem;
	int ret;

#if 1
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
#else
	mem = NULL;
	panel->base = (void __iomem *)0xB3050140;
#endif
	dev_info(&pdev->dev, "Got TVE regs: %p\n", panel->base);

	ret = device_create_file(dev, &dev_attr_tv_norm);
	if (ret)
		return ret;

	panel->dev = dev;

	jz4770_tve_configure_test_bars(panel);

	*out_panel = panel;
	return 0;
}

static void jz4770_tve_panel_exit(void *panel_ptr)
{
	struct jz4770_tve *panel = panel_ptr;

	device_remove_file(panel->dev, &dev_attr_tv_norm);
}

static void jz4770_tve_panel_enable(void *panel_ptr)
{
	struct jz4770_tve *panel = panel_ptr;

	jz4770_tve_set_enabled(panel, true);
}

static void jz4770_tve_panel_disable(void *panel_ptr)
{
	struct jz4770_tve *panel = panel_ptr;

	jz4770_tve_set_enabled(panel, false);
}

struct panel_ops jz4770_tve_panel_ops = {
	.init		= jz4770_tve_panel_init,
	.exit		= jz4770_tve_panel_exit,
	.enable		= jz4770_tve_panel_enable,
	.disable	= jz4770_tve_panel_disable,
};

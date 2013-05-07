/*
 * jz4770_fb.c -- Ingenic Jz4770 LCD frame buffer device
 *
 * Copyright (C) 2012, Maarten ter Huurne <maarten@treewalker.org>
 *
 * Based on the JZ4760 frame buffer driver:
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 * Author: Wolfgang Wang, <lgwang@ingenic.cn>
 *
 * Includes code fragments from JZ4740 SoC LCD frame buffer driver:
 * Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/interrupt.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/processor.h>

#include <asm/mach-jz4770/jz4770_fb.h>
#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770lcdc.h>
#include <asm/mach-jz4770/jz4770misc.h>
#include <asm/mach-jz4770/jz4770tcu.h>

#include <video/jzpanel.h>

#include "console/fbcon.h"


/* use new descriptor(8 words) */
struct jz4760_lcd_dma_desc {
	unsigned int next_desc; 	/* LCDDAx */
	unsigned int databuf;   	/* LCDSAx */
	unsigned int frame_id;  	/* LCDFIDx */
	unsigned int cmd; 		/* LCDCMDx */
	unsigned int offsize;       	/* Stride Offsize(in word) */
	unsigned int page_width; 	/* Stride Pagewidth(in word) */
	unsigned int cmd_num; 		/* Command Number(for SLCD) */
	unsigned int desc_size; 	/* Foreground Size */
};

struct jz4760lcd_panel_t {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int ctrl;	/* lcd controll register */
	unsigned int w;		/* Panel Width(in pixel) */
	unsigned int h;		/* Panel Height(in line) */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};

static const struct jz4760lcd_panel_t jz4760_lcd_panel = {
	.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */
	       LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
	       LCD_CFG_MODE_TFT_24BIT | 	/* output 24bpp */
	       LCD_CFG_HSP | 	/* Hsync polarity: active low */
	       LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
	.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
	/* w, h, fclk, hsw, vsw, elw, blw, efw, bfw */
	320, 240, 60, 50, 1, 17, 70, 9, 13,
	/* Note: Data sheet suggests elw=18 and efw=10, but that doesn't
	 *       line up well with PLL1 at 192 MHz.
	 */
};

/* default output to lcd panel */
static const struct jz4760lcd_panel_t *jz_panel = &jz4760_lcd_panel;

struct jzfb {
	struct fb_info *fb;
	struct jzfb_platform_data *pdata;
	struct platform_device *pdev;
	void *panel;

	uint32_t pseudo_palette[16];
	unsigned int bpp;	/* Current 'bits per pixel' value (32 or 16) */

	struct clk *lpclk;

	struct mutex lock;
	bool is_enabled;
};

static struct jz4760_lcd_dma_desc *dma_desc_base;
static struct jz4760_lcd_dma_desc *dma0_desc0, *dma0_desc1, *dma1_desc0, *dma1_desc1;

#define DMA_DESC_NUM 		6

static unsigned char *lcd_frame1;

static struct jz4760_lcd_dma_desc *dma0_desc_cmd0, *dma0_desc_cmd;

static void ctrl_enable(void)
{
	__lcd_clr_dis();
	__lcd_set_ena();
}

static void ctrl_disable(struct platform_device *pdev)
{
	int cnt;

	/* Use regular disable: finishes current frame, then stops. */
	__lcd_set_dis();

	/* Wait 20 ms for frame to end (at 60 Hz, one frame is 17 ms). */
	for (cnt = 20; cnt > 0 && !__lcd_disable_done(); cnt -= 4)
		msleep(4);
	if (cnt <= 0)
		dev_err(&pdev->dev, "LCD disable timeout! REG_LCD_STATE=0x%08xx\n",
		       REG_LCD_STATE);

	REG_LCD_STATE &= ~LCD_STATE_LDD;
}

static int jz4760fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int transp, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	if (regno >= ARRAY_SIZE(jzfb->pseudo_palette))
		return 1;

	if (fb->var.bits_per_pixel == 16) {
		red	>>= 8;
		green	>>= 8;
		blue	>>= 8;

		red	&= 0xff;
		green	&= 0xff;
		blue	&= 0xff;
	}

	if (fb->var.bits_per_pixel == 16)
		((u32 *)fb->pseudo_palette)[regno] =
				((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3);
	else
		((u32 *)fb->pseudo_palette)[regno] =
				(red << 16) | (green << 8) | (blue << 0);

	return 0;
}

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int jz4760fb_mmap(struct fb_info *fb, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = fb->fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + fb->fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

#if 1
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */
#endif

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

static struct fb_videomode video_modes[] = {
	{
		.name = "320x240",
		.xres = 320,
		.yres = 240,
		// TODO(MtH): Set refresh or pixclock.
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int jz4760fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct fb_videomode *mode = &video_modes[0];
	struct jzfb *jzfb = info->par;

	if (var->bits_per_pixel != 32 && var->bits_per_pixel != 16)
		var->bits_per_pixel = 32;

	if (var->xres != mode->xres)
		var->xres = mode->xres;
	if (var->yres != mode->yres)
		var->yres = mode->yres;


	dev_dbg(&jzfb->pdev->dev, "Found working mode: %s\n", mode->name);

	fb_videomode_to_var(var, mode);

	/* Reserve space for double buffering. */
	//var->yres_virtual = var->yres * 2;
	// TODO(MtH): First try to get it to work without double buffering.
	var->yres_virtual = var->yres;

	if (var->bits_per_pixel == 16) {
		var->blue.length = var->red.length = 5;
		var->green.length = 6;
		var->red.offset = 11;
		var->green.offset = 5;
		var->blue.offset = 0;
	} else {
		var->transp.offset = 24;
		var->red.offset = 16;
		var->green.offset = 8;
		var->blue.offset = 0;
		var->transp.length = var->red.length =
				var->green.length = var->blue.length = 8;
	}

	return 0;
}

static void jzfb_enable(struct jzfb *jzfb)
{
	clk_enable(jzfb->lpclk);

	// TODO: Configure GPIO pins via pinctrl.

	ctrl_enable();
}

static void jzfb_disable(struct jzfb *jzfb)
{
	ctrl_disable(jzfb->pdev);

	// TODO: Configure GPIO pins via pinctrl.

	clk_disable(jzfb->lpclk);
}

/*
 * (Un)blank the display.
 */
static int jz4760fb_blank(int blank_mode, struct fb_info *info)
{
	struct jzfb *jzfb = info->par;

	mutex_lock(&jzfb->lock);

	if (blank_mode == FB_BLANK_UNBLANK) {
		if (!jzfb->is_enabled) {
			jzfb_enable(jzfb);
			jzfb->is_enabled = true;
		}
	} else {
		if (jzfb->is_enabled) {
			jzfb_disable(jzfb);
			jzfb->is_enabled = false;
		}
	}

	mutex_unlock(&jzfb->lock);

	return 0;
}

static int jz4760fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	if (var->xoffset != fb->var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	dev_dbg(&jzfb->pdev->dev, "var.yoffset: %d\n", var->yoffset);
	dma1_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame1
			+ (fb->fix.line_length * var->yoffset));
	dma_cache_wback((unsigned int)(dma1_desc0),
			sizeof(struct jz4760_lcd_dma_desc));

	return 0;
}

/*
 * Map screen memory
 */
static int jz4760fb_map_smem(struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	unsigned long page;
	unsigned int page_shift, needroom, needroom1, w, h;

	dev_dbg(&jzfb->pdev->dev, "FG1 BPP: %d\n", jzfb->bpp);

	w = jz_panel->w;
	h = jz_panel->h;
	needroom1 = needroom = ((w * jzfb->bpp + 7) >> 3) * h;

	for (page_shift = 0; page_shift < 13; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	lcd_frame1 = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
	if (!lcd_frame1)
		return -ENOMEM;

	dma_desc_base = (struct jz4760_lcd_dma_desc *)
			__get_free_pages(GFP_KERNEL, 0);
	if (!dma_desc_base)
		return -ENOMEM;

	memset(lcd_frame1, 0, PAGE_SIZE << page_shift);
	memset(dma_desc_base, 0, PAGE_SIZE);

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	SetPageReserved(virt_to_page(dma_desc_base));
	for (page = (unsigned long)lcd_frame1;
	     page < PAGE_ALIGN((unsigned long)lcd_frame1 + (PAGE_SIZE<<page_shift));
	     page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	fb->fix.smem_start = virt_to_phys((void *)lcd_frame1);
	fb->fix.smem_len = (PAGE_SIZE << page_shift); /* page_shift/2 ??? */
	fb->screen_base =
		(unsigned char *)(((unsigned int)lcd_frame1&0x1fffffff) | 0xa0000000);

	if (!fb->screen_base) {
		dev_err(&jzfb->pdev->dev,
					"%s: unable to map screen memory\n", fb->fix.id);
		return -ENOMEM;
	}

	return 0;
}

static void jz4760fb_unmap_smem(struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	struct page *map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, w, h;

	w = jz_panel->w;
	h = jz_panel->h;
	needroom = ((w * jzfb->bpp + 7) >> 3) * h;

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (fb && fb->screen_base) {
		iounmap(fb->screen_base);
		fb->screen_base = NULL;
		release_mem_region(fb->fix.smem_start, fb->fix.smem_len);
	}

	if (lcd_frame1) {
		for (tmp=(unsigned char *)lcd_frame1;
		     tmp < lcd_frame1 + (PAGE_SIZE << page_shift);
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}
		free_pages((int)lcd_frame1, page_shift);
	}
}

/* initial dma descriptors */
static void jz4760fb_descriptor_init(unsigned int bpp)
{
	dma0_desc0 		= dma_desc_base + 1;
	dma0_desc1 		= dma_desc_base + 2;
	dma0_desc_cmd0 		= dma_desc_base + 3; /* use only once */
	dma0_desc_cmd 		= dma_desc_base + 4;
	dma1_desc0 		= dma_desc_base + 5;
	dma1_desc1 		= dma_desc_base + 6;

	/*
	 * Normal TFT panel's DMA Chan0:
	 *	TO LCD Panel:
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc0
	 * 		palette :	dma0_desc_palette <<==>> dma0_desc0
	 *	TO TV Encoder:
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc1
	 * 		palette:	dma0_desc_palette --> dma0_desc0
	 * 				--> dma0_desc1 --> dma0_desc_palette --> ...
	 *
	 * SMART LCD TFT panel(dma0_desc_cmd)'s DMA Chan0:
	 *	TO LCD Panel:
	 * 		no palette:	dma0_desc_cmd <<==>> dma0_desc0
	 * 		palette :	dma0_desc_palette --> dma0_desc_cmd
	 * 				--> dma0_desc0 --> dma0_desc_palette --> ...
	 *	TO TV Encoder:
	 * 		no palette:	dma0_desc_cmd --> dma0_desc0
	 * 				--> dma0_desc1 --> dma0_desc_cmd --> ...
	 * 		palette:	dma0_desc_palette --> dma0_desc_cmd
	 * 				--> dma0_desc0 --> dma0_desc1
	 * 				--> dma0_desc_palette --> ...
	 * DMA Chan1:
	 *	TO LCD Panel:
	 * 		dma1_desc0 <<==>> dma1_desc0
	 *	TO TV Encoder:
	 * 		dma1_desc0 <<==>> dma1_desc1
	 */

	/* DMA1 Descriptor0 */
	dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
	dma1_desc0->databuf = virt_to_phys((void *)lcd_frame1);
	dma1_desc0->frame_id = (unsigned int)0x0000da10; /* DMA1'0 */

	REG_LCD_DA1 = virt_to_phys(dma1_desc0);	/* set Dma-chan1's Descripter Addrress */
	dma_cache_wback_inv((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4760_lcd_dma_desc));
}

static void jz4760fb_set_panel_mode(struct jzfb *jzfb,
			const struct jz4760lcd_panel_t *panel)
{
	unsigned int bpp = jzfb->bpp;
	unsigned int ctrl = panel->ctrl;
	unsigned int osdctrl = 0;
	unsigned int cfg = panel->cfg;

	if (jzfb->bpp == 16) {
		ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
		osdctrl |= LCD_OSDCTRL_OSDBPP_15_16;
	} else {
		if (WARN_ON(jzfb->bpp != 32))
			jzfb->bpp = 32;

		ctrl |= LCD_CTRL_BPP_18_24;
		osdctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */

	REG_LCD_CTRL = ctrl; /* LCDC Control Register */
	REG_LCD_CFG = cfg; /* LCDC Configure Register */
	REG_SLCD_CFG = 0; /* Smart LCD Configure Register */

	__lcd_vat_set_ht(panel->blw + panel->w + panel->elw);
	__lcd_vat_set_vt(panel->bfw + panel->h + panel->efw);
	__lcd_dah_set_hds(panel->blw);
	__lcd_dah_set_hde(panel->blw + panel->w);
	__lcd_dav_set_vds(panel->bfw);
	__lcd_dav_set_vde(panel->bfw + panel->h);
	__lcd_hsync_set_hps(0);
	__lcd_hsync_set_hpe(panel->hsw);
	__lcd_vsync_set_vpe(panel->vsw);

	REG_LCD_OSDC = LCD_OSDC_F1EN | LCD_OSDC_OSDEN;
	REG_LCD_OSDCTRL = osdctrl;

	/* yellow background helps debugging */
	REG_LCD_BGC = 0x00FFFF00;
}


static void jz4760fb_foreground_resize(const struct jz4760lcd_panel_t *panel, unsigned int bpp)
{
	/* bytes per line, rounded to word boundary */
	int fg1_line_size = ((panel->w * bpp + 31) / 32) * 4;
	/* bytes per frame */
	int fg1_frm_size = fg1_line_size * panel->h;

	/*
	 * NOTE:
	 * Foreground change sequence:
	 * 	1. Change Position Registers -> LCD_OSDCTL.Change;
	 * 	2. LCD_OSDCTRL.Change -> descripter->Size
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size;
	 *	2. F0 position
	 * 	3. F1 size
	 *	4. F1 position
	 */

	REG_LCD_XYP1 = 0;

	/* wait change ready??? */
//		while ( REG_LCD_OSDS & LCD_OSDS_READY )	/* fix in the future, Wolfgang, 06-20-2008 */

	dma1_desc0->cmd = dma1_desc1->cmd = fg1_frm_size/4;
	dma1_desc0->offsize = dma1_desc1->offsize =0;
	dma1_desc0->page_width = dma1_desc1->page_width = 0;

	dma1_desc0->desc_size = dma1_desc1->desc_size
		= panel->h << 16 | panel->w;
	REG_LCD_SIZE1 = (panel->h << 16) | panel->w;

	dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4760_lcd_dma_desc));
}

static void jzfb_change_clock(struct jzfb *jzfb,
			      const struct jz4760lcd_panel_t *panel)
{
	unsigned int pixticks;
	unsigned int rate;
	struct clk *lpclk = jzfb->lpclk;

	pixticks = panel->w;
	if ((panel->cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_SERIAL_TFT)
		pixticks *= 3; /* send R,G,B serially: 8 bits at a time */

	rate = panel->fclk * (pixticks + panel->elw + panel->blw)
	                   * (panel->h + panel->efw + panel->bfw);

	clk_disable(lpclk);

	/* Use pixel clock for LCD panel (as opposed to TV encoder). */
	__cpm_select_pixclk_lcd();

	clk_set_rate(lpclk, rate);

	jz_clocks.pixclk = clk_get_rate(lpclk);
	dev_dbg(&jzfb->pdev->dev, "PixClock: %d\n", jz_clocks.pixclk);

	clk_enable(lpclk);

	/*
	 * Enabling the LCDC too soon after the clock will hang the system.
	 * A very short delay seems to be sufficient.
	 */
	udelay(1);
}

/* set the video mode according to info->var */
static int jz4760fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct jzfb *jzfb = info->par;

	__lcd_clr_ena(); /* quick disable */

	jzfb->bpp = var->bits_per_pixel;
	jz4760fb_set_panel_mode(jzfb, jz_panel);
	jz4760fb_foreground_resize(jz_panel, jzfb->bpp);
	jzfb_change_clock(jzfb, jz_panel);

	ctrl_enable();

	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->line_length = var->xres_virtual * (var->bits_per_pixel >> 3);
	return 0;
}

static struct fb_ops jz4760fb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= jz4760fb_setcolreg,
	.fb_check_var 		= jz4760fb_check_var,
	.fb_set_par 		= jz4760fb_set_par,
	.fb_blank		= jz4760fb_blank,
	.fb_pan_display		= jz4760fb_pan_display,
	.fb_fillrect		= sys_fillrect,
	.fb_copyarea		= sys_copyarea,
	.fb_imageblit		= sys_imageblit,
	.fb_mmap		= jz4760fb_mmap,
};

static irqreturn_t jz4760fb_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;
	static int irqcnt=0;

	state = REG_LCD_STATE;
	pr_debug("In the lcd interrupt handler, state=0x%x\n", state);

	if (state & LCD_STATE_EOF) /* End of frame */
		REG_LCD_STATE = state & ~LCD_STATE_EOF;

	if (state & LCD_STATE_IFU0) {
		pr_warn("%s, InFiFo0 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_IFU1) {
		pr_warn("%s, InFiFo1 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU1;
	}

	if (state & LCD_STATE_OFU) { /* Out fifo underrun */
		REG_LCD_STATE = state & ~LCD_STATE_OFU;
		if ( irqcnt++ > 100 ) {
			__lcd_disable_ofu_intr();
			pr_debug("disable Out FiFo underrun irq.\n");
		}
		pr_warn("%s, Out FiFo underrun.\n", __FUNCTION__);
	}

	return IRQ_HANDLED;
}

static void gpio_init(void)
{
	/* gpio init __gpio_as_lcd */
	if (jz_panel->cfg & LCD_CFG_MODE_TFT_16BIT)
		__gpio_as_lcd_16bit();
	else if (jz_panel->cfg & LCD_CFG_MODE_TFT_24BIT)
		__gpio_as_lcd_24bit();
	else
		__gpio_as_lcd_18bit();
}

static int jz4760_fb_probe(struct platform_device *pdev)
{
	struct jzfb_platform_data *pdata = pdev->dev.platform_data;
	struct jzfb *jzfb;
	struct fb_info *fb;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "Missing platform data\n");
		return -ENXIO;
	}

	fb = framebuffer_alloc(sizeof(struct jzfb), &pdev->dev);
	if (!fb) {
		dev_err(&pdev->dev, "Failed to allocate framebuffer device\n");
		return -ENOMEM;
	}

	jzfb = fb->par;
	jzfb->pdev = pdev;
	jzfb->pdata = pdata;
	jzfb->bpp = 32;

	strcpy(fb->fix.id, "jz-lcd");
	fb->fix.type	= FB_TYPE_PACKED_PIXELS;
	fb->fix.type_aux	= 0;
	fb->fix.xpanstep	= 1;
	fb->fix.ypanstep	= 1;
	fb->fix.ywrapstep	= 0;
	fb->fix.accel	= FB_ACCEL_NONE;

	fb->var.nonstd	= 0;
	fb->var.activate	= FB_ACTIVATE_NOW;
	fb->var.height	= -1;
	fb->var.width	= -1;
	fb->var.accel_flags	= FB_ACCELF_TEXT;
	fb->var.bits_per_pixel = jzfb->bpp;

	jz4760fb_check_var(&fb->var, fb);

	fb->fbops		= &jz4760fb_ops;
	fb->flags		= FBINFO_FLAG_DEFAULT;

	fb->pseudo_palette	= jzfb->pseudo_palette;

	gpio_init();

	ret = jz4760fb_map_smem(fb);
	if (ret)
		goto failed;

	/* Init pixel clock. */
	jzfb->lpclk = devm_clk_get(&pdev->dev, "lpclk");
	if (IS_ERR(jzfb->lpclk)) {
		ret = PTR_ERR(jzfb->lpclk);
		dev_err(&pdev->dev, "Failed to get pixel clock: %d\n", ret);
		goto failed;
	}

	if (request_irq(IRQ_LCD, jz4760fb_interrupt_handler, IRQF_DISABLED,
				"lcd", 0)) {
		dev_err(&pdev->dev, "Failed to request IRQ.\n");
		ret = -EBUSY;
		goto failed;
	}

	platform_set_drvdata(pdev, jzfb);

	/* configurate sequence:
	 * 1. disable lcdc.
	 * 2. init frame descriptor.
	 * 3. set panel mode
	 * 4. set osd mode
	 * 5. start lcd clock in CPM
	 * 6. enable lcdc.
	 */

	/*
	 * We expect the LCDC to be disabled when this driver is loaded,
	 * since having DMA activity going on outside of the kernel's
	 * knowledge is asking for trouble. Unfortunately, there does not
	 * seem to be a way to check whether the LCDC is currently enabled.
	 * So we do a few harmless writes just in case it was left in a
	 * not fully disabled state for some reason.
	 */
	REG_LCD_STATE = 0; /* clear lcdc status */
	__lcd_clr_ena(); /* quick disable */

	jz4760fb_descriptor_init(jzfb->bpp);
	jz4760fb_set_par(fb);

	ret = register_framebuffer(fb);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer device.\n");
		goto failed;
	}
	dev_info(&pdev->dev,
				"fb%d: %s frame buffer device, using %dK of video memory\n",
				fb->node, fb->fix.id, fb->fix.smem_len>>10);

	mutex_init(&jzfb->lock);

	jzfb->fb = fb;

	// TODO: Panels should be proper modules that register themselves.
	//       They should be switchable via sysfs.
	//       And a module parameter should select the default panel.

	ret = pdata->panel_ops->init(&jzfb->panel,
				     &pdev->dev, pdata->panel_pdata);
	if (ret)
		goto failed;

	pdata->panel_ops->enable(jzfb->panel);
	jzfb->is_enabled = true;

	return 0;

failed:
	jz4760fb_unmap_smem(fb);
	framebuffer_release(fb);

	return ret;
}

static int jz4760_fb_remove(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	if (jzfb->is_enabled) {
		jzfb_disable(jzfb);
		jzfb->pdata->panel_ops->disable(jzfb->panel);
	}

	jzfb->pdata->panel_ops->exit(jzfb->panel);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM

static int jz4760_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "Suspending\n");

	if (jzfb->is_enabled) {
		jzfb_disable(jzfb);
		jzfb->pdata->panel_ops->disable(jzfb->panel);
	}

	return 0;
}

static int jz4760_fb_resume(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "Resuming\n");

	if (jzfb->is_enabled) {
		jzfb->pdata->panel_ops->enable(jzfb->panel);
		jzfb_enable(jzfb);
	}

	return 0;
}

#else
#define jz4760_fb_suspend	NULL
#define jz4760_fb_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver jz4760_fb_driver = {
	.probe	= jz4760_fb_probe,
	.remove = jz4760_fb_remove,
	.suspend = jz4760_fb_suspend,
	.resume = jz4760_fb_resume,
	.driver = {
		.name = "jz-lcd",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(jz4760_fb_driver);

MODULE_DESCRIPTION("Jz4770 LCD frame buffer driver");
MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_LICENSE("GPL");

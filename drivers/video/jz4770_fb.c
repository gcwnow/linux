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
#include <linux/wait.h>

#include <asm/addrspace.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/processor.h>

#include <asm/mach-jz4770/jz4770_fb.h>
#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/jz4770lcdc.h>
#include <asm/mach-jz4770/jz4770misc.h>

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
	       LCD_CFG_PCP |	/* Pixel clock polarity: falling edge */
	       LCD_CFG_HSP | 	/* Hsync polarity: active low */
	       LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
	/* w, h, fclk, hsw, vsw, elw, blw, efw, bfw */
	320, 240, 60, 50, 1, 10, 70, 5, 5,
	/* Note: 432000000 / 72 = 60 * 400 * 250, so we get exactly 60 Hz. */
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

	uint32_t pan_offset;
	uint32_t vsync_count;
	wait_queue_head_t wait_vsync;

	struct clk *lpclk;

	struct mutex lock;
	bool is_enabled;
	/*
	 * Number of frames to wait until doing a forced foreground flush.
	 * If it looks like we are double buffering, we can flush on vertical
	 * panning instead.
	 */
	unsigned int delay_flush;

	void __iomem *base;
};

static struct jz4760_lcd_dma_desc dma1_desc0 __aligned(16),
								  dma1_desc1 __aligned(16);
static void *lcd_frame1;

static void ctrl_enable(struct jzfb *jzfb)
{
	u32 val = readl(jzfb->base + LCD_CTRL);
	val = (val & ~LCD_CTRL_DIS) | LCD_CTRL_ENA;
	writel(val, jzfb->base + LCD_CTRL);
}

static void ctrl_disable(struct jzfb *jzfb)
{
	unsigned int cnt;
	u32 val;

	/* Use regular disable: finishes current frame, then stops. */
	val = readl(jzfb->base + LCD_CTRL) | LCD_CTRL_DIS;
	writel(val, jzfb->base + LCD_CTRL);

	/* Wait 20 ms for frame to end (at 60 Hz, one frame is 17 ms). */
	for (cnt = 20; cnt; cnt -= 4) {
		if (readl(jzfb->base + LCD_STATE) & LCD_STATE_LDD)
			break;
		msleep(4);
	}
	if (!cnt)
		dev_err(&jzfb->pdev->dev, "LCD disable timeout!\n");

	val = readl(jzfb->base + LCD_STATE);
	writel(val & ~LCD_STATE_LDD, jzfb->base + LCD_STATE);
}

static int jz4760fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int transp, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	if (regno >= ARRAY_SIZE(jzfb->pseudo_palette))
		return 1;

	if (fb->var.bits_per_pixel == 16)
		((u32 *)fb->pseudo_palette)[regno] =
				(red & 0xf800) | ((green & 0xfc00) >> 5) | (blue >> 11);
	else
		((u32 *)fb->pseudo_palette)[regno] =
				((red & 0xff00) << 8) | (green & 0xff00) | (blue >> 8);

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

	/* Set cacheability to cacheable, write through, no write-allocate. */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;

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
	struct jzfb *jzfb = info->par;
	const struct fb_videomode *mode;

	mode = fb_find_best_mode(var, &info->modelist);
	if (!mode)
		return -EINVAL;
	dev_dbg(&jzfb->pdev->dev, "Found working mode: %s\n", mode->name);

	fb_videomode_to_var(var, mode);
	/* Reserve space for double buffering. */
	var->yres_virtual = var->yres * 3;

	if (var->bits_per_pixel != 32 && var->bits_per_pixel != 16)
		var->bits_per_pixel = 32;

	if (var->bits_per_pixel == 16) {
		var->transp.length = 0;
		var->blue.length = var->red.length = 5;
		var->green.length = 6;
		var->transp.offset = 0;
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

static int jzfb_wait_for_vsync(struct jzfb *jzfb)
{
	uint32_t count = jzfb->vsync_count;
	long t = wait_event_interruptible_timeout(jzfb->wait_vsync,
						  count != jzfb->vsync_count,
						  HZ / 10);
	return t > 0 ? 0 : (t < 0 ? (int)t : -ETIMEDOUT);
}

static void jzfb_update_frame_address(struct jzfb *jzfb)
{
	unsigned int addr = (unsigned int) virt_to_phys(lcd_frame1)
			    + jzfb->pan_offset;
	if (dma1_desc0.databuf != addr) {
		dma1_desc0.databuf = addr;
		dma_cache_wback((unsigned int) &dma1_desc0, sizeof(dma1_desc0));
	}
}

static void jzfb_lcdc_enable(struct jzfb *jzfb)
{
	clk_enable(jzfb->lpclk);
	jzfb_update_frame_address(jzfb);

	writel(virt_to_phys(&dma1_desc1), jzfb->base + LCD_DA1);
	jzfb->delay_flush = 0;

	writel(0, jzfb->base + LCD_STATE); /* Clear LCDC status */

	/*
	 * Enabling the LCDC too soon after the clock will hang the system.
	 * A very short delay seems to be sufficient.
	 */
	udelay(1);

	ctrl_enable(jzfb);
}

static void jzfb_lcdc_disable(struct jzfb *jzfb)
{
	ctrl_disable(jzfb);

	clk_disable(jzfb->lpclk);
}

static void jzfb_power_up(struct jzfb *jzfb)
{
	// TODO: Configure GPIO pins via pinctrl.

	jzfb->pdata->panel_ops->enable(jzfb->panel);

	jzfb_lcdc_enable(jzfb);
}

static void jzfb_power_down(struct jzfb *jzfb)
{
	jzfb_lcdc_disable(jzfb);

	jzfb->pdata->panel_ops->disable(jzfb->panel);

	// TODO: Configure GPIO pins via pinctrl.
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
			jzfb_power_up(jzfb);
			jzfb->is_enabled = true;
		}
	} else {
		if (jzfb->is_enabled) {
			jzfb_power_down(jzfb);
			jzfb->is_enabled = false;
		}
	}

	mutex_unlock(&jzfb->lock);

	return 0;
}

static int jz4760fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	uint32_t vpan = var->yoffset;

	if (var->xoffset != fb->var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	jzfb->pan_offset = fb->fix.line_length * vpan;
	dev_dbg(&jzfb->pdev->dev, "var.yoffset: %d\n", vpan);

	jzfb->delay_flush = 8;
	dma_cache_wback_inv((unsigned long)(lcd_frame1 + jzfb->pan_offset),
			    fb->fix.line_length * var->yres);

	/*
	 * The primary use of this function is to implement double buffering.
	 * Explicitly waiting for vsync and then panning doesn't work in
	 * practice because the woken up process doesn't always run before the
	 * next frame has already started: the time between vsync and the start
	 * of the next frame is typically less than one scheduler time slice.
	 * Instead, we wait for vsync here in the pan function and apply the
	 * new panning setting in the vsync interrupt, so we know that the new
	 * panning setting has taken effect before this function returns.
	 * Note that fb->var is only updated after we return, so we need our
	 * own copy of the panning setting (jzfb->pan_offset).
	 */
	jzfb_wait_for_vsync(jzfb);

	return 0;
}

static inline unsigned int words_per_line(unsigned int width, unsigned int bpp)
{
	return (bpp * width + 31) / 32;
}

static unsigned int max_bytes_per_frame(struct list_head *modelist,
					unsigned int bpp)
{
	unsigned int max_words = 0;
	struct fb_modelist *pos;

	list_for_each_entry(pos, modelist, list) {
		struct fb_videomode *m = &pos->mode;
		unsigned int words = words_per_line(m->xres, bpp) * m->yres;
		if (words > max_words)
			max_words = words;
	}

	return max_words * 4;
}

/*
 * Map screen memory
 */
static int jz4760fb_map_smem(struct fb_info *fb)
{
	/* Compute space for max res at 32bpp, triple buffered. */
	const unsigned int size = PAGE_ALIGN(
				max_bytes_per_frame(&fb->modelist, 32) * 3);

	void *page_virt;

	dev_dbg(fb->device, "FG1: %u bytes\n", size);

	lcd_frame1 = alloc_pages_exact(size, GFP_KERNEL);
	if (!lcd_frame1) {
		dev_err(fb->device,
			"Unable to map %u bytes of screen memory\n", size);
		return -ENOMEM;
	}

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	for (page_virt = lcd_frame1;
	     page_virt < lcd_frame1 + size; page_virt += PAGE_SIZE) {
		SetPageReserved(virt_to_page(page_virt));
		clear_page(page_virt);
	}

	fb->fix.smem_start = virt_to_phys(lcd_frame1);
	fb->fix.smem_len = size;
	fb->screen_base = (void *)KSEG1ADDR(lcd_frame1);

	return 0;
}

static void jz4760fb_unmap_smem(struct fb_info *fb)
{
	if (lcd_frame1) {
		void *end = lcd_frame1 + fb->fix.smem_len;
		void *page_virt;

		for (page_virt = lcd_frame1; page_virt < end;
							page_virt += PAGE_SIZE)
			ClearPageReserved(virt_to_page(page_virt));

		free_pages_exact(lcd_frame1, fb->fix.smem_len);
	}
}

/* initial dma descriptors */
static void jz4760fb_descriptor_init(unsigned int bpp)
{
	/* DMA1 Descriptor0 */
	dma1_desc0.next_desc = (unsigned int) virt_to_phys(&dma1_desc1);
	dma1_desc0.databuf = virt_to_phys(lcd_frame1);
	dma1_desc0.cmd = LCD_CMD_SOFINT | LCD_CMD_EOFINT;

	/*
	 * DMA1 Descriptor1
	 * This is a single invisible line that we place just before the start
	 * of the frame, to force Descriptor0 to be loaded at the last possible
	 * moment, to make sure any panning changes (for double buffering) will
	 * take effect in the next frame rather than the one after that.
	 */
	dma1_desc1.next_desc = (unsigned int) virt_to_phys(&dma1_desc0);
	dma1_desc1.databuf = virt_to_phys(lcd_frame1);

	dma1_desc0.offsize = dma1_desc1.offsize = 0;
	dma1_desc0.page_width = dma1_desc1.page_width = 0;

	dma_cache_wback_inv((unsigned int) &dma1_desc0, sizeof(dma1_desc0));
	dma_cache_wback_inv((unsigned int) &dma1_desc1, sizeof(dma1_desc1));
}

static void jz4760fb_set_panel_mode(struct jzfb *jzfb,
			const struct jz4760lcd_panel_t *panel)
{
	unsigned int bpp = jzfb->bpp;
	u32 cfg = panel->cfg;
	u16 osdctrl = 0;

	if (bpp == 16) {
		osdctrl |= LCD_OSDCTRL_OSDBPP_15_16;
	} else {
		if (WARN_ON(bpp != 32))
			bpp = 32;

		osdctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */

	writel(cfg, jzfb->base + LCD_CFG); /* LCDC Configure Register */

	/* 16 words burst, enable output FIFO underrun IRQ, 18/24 bpp on fg0. */
	writel(LCD_CTRL_OFUM | LCD_CTRL_BST_16 | LCD_CTRL_BPP_18_24,
			jzfb->base + LCD_CTRL);

	writel((panel->blw + panel->w + panel->elw) << LCD_VAT_HT_BIT |
			(panel->bfw + panel->h + panel->efw) << LCD_VAT_VT_BIT,
		jzfb->base + LCD_VAT);
	writel(panel->blw << LCD_DAH_HDS_BIT |
			(panel->blw + panel->w) << LCD_DAH_HDE_BIT,
			jzfb->base + LCD_DAH);
	writel((panel->bfw - 1) << LCD_DAV_VDS_BIT |
			(panel->bfw + panel->h) << LCD_DAV_VDE_BIT,
			jzfb->base + LCD_DAV);

	writel(panel->hsw << LCD_HSYNC_HPE_BIT, jzfb->base + LCD_HSYNC);
	writel(panel->vsw << LCD_VSYNC_VPE_BIT, jzfb->base + LCD_VSYNC);

	/* Enable foreground 1, OSD mode,
	 * start-of-frame and end-of-frame interrupts */
	writew(LCD_OSDC_F1EN | LCD_OSDC_OSDEN | LCD_OSDC_SOFM1 | LCD_OSDC_EOFM1,
			jzfb->base + LCD_OSDC);
	writew(osdctrl, jzfb->base + LCD_OSDCTRL);

	/* yellow background helps debugging */
	writel(0x00FFFF00, jzfb->base + LCD_BGC);
}


static void jz4760fb_foreground_resize(struct jzfb *jzfb,
		const struct jz4760lcd_panel_t *panel,
		const struct fb_var_screeninfo *var)
{
	int fg1_words_per_line = words_per_line(var->xres, var->bits_per_pixel);
	int fg1_words_per_frame = fg1_words_per_line * var->yres;
	int xpos, ypos;

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

	xpos = (panel->w - var->xres) / 2;
	if (xpos < 0) xpos = 0;
	ypos = (panel->h - var->yres) / 2;
	if (ypos < 0) ypos = 0;

	writel(ypos << 16 | xpos, jzfb->base + LCD_XYP1);

	/* wait change ready??? */
//		while ( REG_LCD_OSDS & LCD_OSDS_READY )	/* fix in the future, Wolfgang, 06-20-2008 */

	dma1_desc0.cmd = (dma1_desc0.cmd & 0xff000000) | fg1_words_per_frame;
	dma1_desc1.cmd = (dma1_desc1.cmd & 0xff000000) | fg1_words_per_line;
	dma1_desc0.desc_size = dma1_desc1.desc_size =
				(var->yres + 1) << 16 | var->xres;

	dma_cache_wback((unsigned int) &dma1_desc0, sizeof(dma1_desc0));
	dma_cache_wback((unsigned int) &dma1_desc1, sizeof(dma1_desc1));
}

static void jzfb_change_clock(struct jzfb *jzfb,
			      const struct jz4760lcd_panel_t *panel)
{
	unsigned int rate;

	rate = panel->fclk * (panel->w + panel->elw + panel->blw)
	                   * (panel->h + panel->efw + panel->bfw);

	/* Use pixel clock for LCD panel (as opposed to TV encoder). */
	__cpm_select_pixclk_lcd();

	clk_set_rate(jzfb->lpclk, rate);

	dev_dbg(&jzfb->pdev->dev, "PixClock: req %u, got %lu\n",
		rate, clk_get_rate(jzfb->lpclk));
}

/* set the video mode according to info->var */
static int jz4760fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct jzfb *jzfb = info->par;

	if (jzfb->is_enabled)
		ctrl_disable(jzfb);
	else
		clk_enable(jzfb->lpclk);

	jzfb->bpp = var->bits_per_pixel;
	jz4760fb_set_panel_mode(jzfb, jz_panel);
	jz4760fb_foreground_resize(jzfb, jz_panel, var);

	clk_disable(jzfb->lpclk);

	jzfb_change_clock(jzfb, jz_panel);

	if (jzfb->is_enabled)
		jzfb_lcdc_enable(jzfb);

	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->line_length = var->xres_virtual * (var->bits_per_pixel >> 3);
	return 0;
}

static int jz4760fb_ioctl(struct fb_info *info, unsigned int cmd,
			  unsigned long arg)
{
	struct jzfb *jzfb = info->par;

	switch (cmd) {
		case FBIO_WAITFORVSYNC:
			return jzfb_wait_for_vsync(jzfb);
		default:
			return -ENOIOCTLCMD;
	}
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
	.fb_ioctl		= jz4760fb_ioctl,
};

static irqreturn_t jz4760fb_interrupt_handler(int irq, void *dev_id)
{
	static int irqcnt = 0;
	struct jzfb *jzfb = dev_id;
	u32 state;
	u16 osds;

	state = readl(jzfb->base + LCD_STATE);
	pr_debug("In the lcd interrupt handler, state=0x%x\n", state);

	if (state & LCD_STATE_SOF) /* Start of frame */
		state &= ~LCD_STATE_SOF;

	if (state & LCD_STATE_EOF) /* End of frame */
		state &= ~LCD_STATE_EOF;

	if (state & LCD_STATE_IFU0) {
		pr_warn("%s, InFiFo0 underrun\n", __FUNCTION__);
		state &= ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_IFU1) {
		pr_warn("%s, InFiFo1 underrun\n", __FUNCTION__);
		state &= ~LCD_STATE_IFU1;
	}

	if (state & LCD_STATE_OFU) { /* Out fifo underrun */
		state &= ~LCD_STATE_OFU;
		if ( irqcnt++ > 100 ) {
			u32 val = readl(jzfb->base + LCD_CTRL) & ~LCD_CTRL_OFUM;
			writel(val, jzfb->base + LCD_CTRL);
			pr_debug("disable Out FiFo underrun irq.\n");
		}
		pr_warn("%s, Out FiFo underrun.\n", __FUNCTION__);
	}

	writel(state, jzfb->base + LCD_STATE);

	osds = readl(jzfb->base + LCD_OSDS);

	if (osds & LCD_OSDS_SOF1) {
		if (jzfb->delay_flush == 0) {
			struct fb_info *fb = jzfb->fb;
			dma_cache_wback_inv((unsigned long)(lcd_frame1 +
							    jzfb->pan_offset),
					fb->fix.line_length * fb->var.yres);
		} else {
			jzfb->delay_flush--;
		}
		osds &= ~LCD_OSDS_SOF1;
	}

	if (osds & LCD_OSDS_EOF1) {
		jzfb_update_frame_address(jzfb);

		jzfb->vsync_count++;
		wake_up_interruptible_all(&jzfb->wait_vsync);

		osds &= ~LCD_OSDS_EOF1;
	}

	writel(osds, jzfb->base + LCD_OSDS);

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
	struct resource *res;
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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	jzfb->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jzfb->base)) {
		dev_err(&pdev->dev, "Failed to request registers\n");
		ret = PTR_ERR(jzfb->base);
		goto err_release_fb;
	}

	jzfb->pdev = pdev;
	jzfb->pdata = pdata;
	jzfb->bpp = 32;
	init_waitqueue_head(&jzfb->wait_vsync);

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

	fb_videomode_to_modelist(video_modes, ARRAY_SIZE(video_modes),
				 &fb->modelist);
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

	if (request_irq(IRQ_LCD, jz4760fb_interrupt_handler, 0,
				"lcd", jzfb)) {
		dev_err(&pdev->dev, "Failed to request IRQ.\n");
		ret = -EBUSY;
		goto failed;
	}

	mutex_init(&jzfb->lock);

	platform_set_drvdata(pdev, jzfb);
	jzfb->fb = fb;

	/*
	 * We assume the LCDC is disabled initially. If you really must have
	 * video in your boot loader, you'll have to update this driver.
	 */

	jz4760fb_descriptor_init(jzfb->bpp);
	jz4760fb_set_par(fb);

	// TODO: Panels should be proper modules that register themselves.
	//       They should be switchable via sysfs.
	//       And a module parameter should select the default panel.

	ret = pdata->panel_ops->init(&jzfb->panel,
				     &pdev->dev, pdata->panel_pdata);
	if (ret)
		goto failed;

	jzfb_power_up(jzfb);
	jzfb->is_enabled = true;

	ret = register_framebuffer(fb);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer device.\n");
		goto err_exit_panel;
	}
	dev_info(&pdev->dev,
		"fb%d: %s frame buffer device, using %dK of video memory\n",
		fb->node, fb->fix.id, fb->fix.smem_len>>10);

	/* XXX(pcercuei): For some reason, this fixes the logo not being shown */
	ctrl_disable(jzfb);
	ctrl_enable(jzfb);

	fb_prepare_logo(jzfb->fb, 0);
	fb_show_logo(jzfb->fb, 0);
	return 0;

err_exit_panel:
	jzfb->pdata->panel_ops->exit(jzfb->panel);
failed:
	jz4760fb_unmap_smem(fb);
err_release_fb:
	framebuffer_release(fb);
	return ret;
}

static int jz4760_fb_remove(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	if (jzfb->is_enabled)
		jzfb_power_down(jzfb);

	jzfb->pdata->panel_ops->exit(jzfb->panel);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int jz4760_fb_suspend(struct device *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	dev_dbg(dev, "Suspending\n");

	if (jzfb->is_enabled)
		jzfb_power_down(jzfb);

	return 0;
}

static int jz4760_fb_resume(struct device *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	dev_dbg(dev, "Resuming\n");

	if (jzfb->is_enabled)
		jzfb_power_up(jzfb);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(jz4760_fb_pm_ops, jz4760_fb_suspend, jz4760_fb_resume);

static struct platform_driver jz4760_fb_driver = {
	.probe	= jz4760_fb_probe,
	.remove = jz4760_fb_remove,
	.driver = {
		.name  = "jz-lcd",
		.owner = THIS_MODULE,
		.pm    = &jz4760_fb_pm_ops,
	},
};

module_platform_driver(jz4760_fb_driver);

MODULE_DESCRIPTION("Jz4770 LCD frame buffer driver");
MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_LICENSE("GPL");

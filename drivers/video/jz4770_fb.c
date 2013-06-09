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

#include <asm/irq.h>
#include <asm/page.h>
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
};

static struct jz4760_lcd_dma_desc dma1_desc0 __aligned(16),
								  dma1_desc1 __aligned(16);
static void *lcd_frame1;

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
	var->yres_virtual = var->yres * 2;

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

	REG_LCD_DA1 = virt_to_phys(&dma1_desc1);
	jzfb->delay_flush = 0;

	REG_LCD_STATE = 0; /* clear lcdc status */

	/*
	 * Enabling the LCDC too soon after the clock will hang the system.
	 * A very short delay seems to be sufficient.
	 */
	udelay(1);

	ctrl_enable();
}

static void jzfb_lcdc_disable(struct jzfb *jzfb)
{
	ctrl_disable(jzfb->pdev);

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

/*
 * Map screen memory
 */
static int jz4760fb_map_smem(struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	unsigned int w = fb->var.xres_virtual,
				 h = fb->var.yres_virtual,
				 bytes_per_frame = words_per_line(w, jzfb->bpp) * h * 4;
	void *page_virt;

	dev_dbg(&jzfb->pdev->dev, "FG1 BPP: %d\n", jzfb->bpp);

	lcd_frame1 = alloc_pages_exact(bytes_per_frame, GFP_KERNEL);
	if (!lcd_frame1) {
		dev_err(&jzfb->pdev->dev,
					"%s: unable to map screen memory\n", fb->fix.id);
		return -ENOMEM;
	}

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	for (page_virt = lcd_frame1;
	     page_virt < lcd_frame1 + bytes_per_frame; page_virt += PAGE_SIZE) {
		SetPageReserved(virt_to_page(page_virt));
		clear_page(page_virt);
	}

	fb->fix.smem_start = virt_to_phys(lcd_frame1);
	fb->fix.smem_len = bytes_per_frame;
	fb->screen_base =
		(unsigned char *)(((unsigned int)lcd_frame1&0x1fffffff) | 0xa0000000);

	return 0;
}

static void jz4760fb_unmap_smem(struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	unsigned int w = fb->var.xres_virtual,
				 h = fb->var.yres_virtual,
				 bytes_per_frame = words_per_line(w, jzfb->bpp) * h * 4;

	if (fb && fb->screen_base) {
		iounmap(fb->screen_base);
		fb->screen_base = NULL;
		release_mem_region(fb->fix.smem_start, fb->fix.smem_len);
	}

	if (lcd_frame1) {
		void *page_virt;

		for (page_virt = lcd_frame1;
			 page_virt < lcd_frame1 + bytes_per_frame; page_virt += PAGE_SIZE)
			ClearPageReserved(virt_to_page(page_virt));

		free_pages_exact(lcd_frame1, bytes_per_frame);
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
	unsigned int ctrl = panel->ctrl;
	unsigned int osdctrl = 0;
	unsigned int cfg = panel->cfg;

	if (bpp == 16) {
		ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
		osdctrl |= LCD_OSDCTRL_OSDBPP_15_16;
	} else {
		if (WARN_ON(bpp != 32))
			bpp = 32;

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
	__lcd_dav_set_vds(panel->bfw - 1);
	__lcd_dav_set_vde(panel->bfw + panel->h);
	__lcd_hsync_set_hps(0);
	__lcd_hsync_set_hpe(panel->hsw);
	__lcd_vsync_set_vpe(panel->vsw);

	REG_LCD_OSDC = LCD_OSDC_F1EN | LCD_OSDC_OSDEN |
		       LCD_OSDC_SOFM1 | LCD_OSDC_EOFM1;
	REG_LCD_OSDCTRL = osdctrl;

	/* yellow background helps debugging */
	REG_LCD_BGC = 0x00FFFF00;
}


static void jz4760fb_foreground_resize(const struct jz4760lcd_panel_t *panel, unsigned int bpp)
{
	int fg1_words_per_line = words_per_line(panel->w, bpp);
	int fg1_words_per_frame = fg1_words_per_line * panel->h;

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

	dma1_desc0.cmd = (dma1_desc0.cmd & 0xff000000) | fg1_words_per_frame;
	dma1_desc1.cmd = (dma1_desc1.cmd & 0xff000000) | fg1_words_per_line;
	dma1_desc0.desc_size = dma1_desc1.desc_size =
				(panel->h + 1) << 16 | panel->w;

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

	jz_clocks.pixclk = clk_get_rate(jzfb->lpclk);
	dev_dbg(&jzfb->pdev->dev, "PixClock: %d\n", jz_clocks.pixclk);
}

/* set the video mode according to info->var */
static int jz4760fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct jzfb *jzfb = info->par;

	if (jzfb->is_enabled)
		ctrl_disable(jzfb->pdev);
	else
		clk_enable(jzfb->lpclk);

	jzfb->bpp = var->bits_per_pixel;
	jz4760fb_set_panel_mode(jzfb, jz_panel);
	jz4760fb_foreground_resize(jz_panel, jzfb->bpp);

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
	unsigned int state;
	static int irqcnt = 0;
	struct jzfb *jzfb = dev_id;

	state = REG_LCD_STATE;
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
			__lcd_disable_ofu_intr();
			pr_debug("disable Out FiFo underrun irq.\n");
		}
		pr_warn("%s, Out FiFo underrun.\n", __FUNCTION__);
	}

	REG_LCD_STATE = state;

	state = REG_LCD_OSDS;

	if (state & LCD_OSDS_SOF1) {
		if (jzfb->delay_flush == 0) {
			struct fb_info *fb = jzfb->fb;
			dma_cache_wback_inv((unsigned long)(lcd_frame1 +
							    jzfb->pan_offset),
					fb->fix.line_length * fb->var.yres);
		} else {
			jzfb->delay_flush--;
		}
		REG_LCD_OSDS &= ~LCD_OSDS_SOF1;
	}

	if (state & LCD_OSDS_EOF1) {
		jzfb_update_frame_address(jzfb);

		jzfb->vsync_count++;
		wake_up_interruptible_all(&jzfb->wait_vsync);

		REG_LCD_OSDS &= ~LCD_OSDS_EOF1;
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

	return 0;

err_exit_panel:
	jzfb->pdata->panel_ops->exit(jzfb->panel);
failed:
	jz4760fb_unmap_smem(fb);
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

#ifdef CONFIG_PM

static int jz4760_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "Suspending\n");

	if (jzfb->is_enabled)
		jzfb_power_down(jzfb);

	return 0;
}

static int jz4760_fb_resume(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "Resuming\n");

	if (jzfb->is_enabled)
		jzfb_power_up(jzfb);

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

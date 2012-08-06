/*
 * jz4770_fb.c -- Ingenic Jz4770 LCD frame buffer device
 *
 * Copyright (C) 2012, Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2014, Paul Cercueil <paul@crapouillou.net>
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
#include <linux/gcd.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
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

#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770misc.h>

#include "jz4770_ipu.h"
#include "jz4770_lcdc.h"

#define MAX_XRES 640
#define MAX_YRES 480

struct jz_panel {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int bw;	/* panel background width (in pixels) */
	unsigned int bh;	/* panel background height (in lines) */
	unsigned int dw;	/* panel display area width (in pixels) */
	unsigned int dh;	/* panel display area height (in lines) */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};

static const struct jz_panel jz4770_lcd_panel = {
	.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |	/* Underrun recover */
		LCD_CFG_MODE_GENERIC_TFT |	/* General TFT panel */
		LCD_CFG_MODE_TFT_24BIT |	/* output 24bpp */
		LCD_CFG_PCP |	/* Pixel clock polarity: falling edge */
		LCD_CFG_HSP |	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
	/* bw, bh, dw, dh, fclk, hsw, vsw, elw, blw, efw, bfw */
	320, 240, 320, 240, 60, 50, 1, 10, 70, 5, 5,
	/* Note: 432000000 / 72 = 60 * 400 * 250, so we get exactly 60 Hz. */
};

struct jzfb {
	struct fb_info *fb;
	struct platform_device *pdev;

	uint32_t pseudo_palette[16];
	unsigned int bpp;	/* Current 'bits per pixel' value (32 or 16) */

	uint32_t pan_offset;
	uint32_t vsync_count;
	wait_queue_head_t wait_vsync;

	struct clk *lpclk, *ipuclk;

	struct mutex lock;
	bool is_enabled;
	/*
	 * Number of frames to wait until doing a forced foreground flush.
	 * If it looks like we are double buffering, we can flush on vertical
	 * panning instead.
	 */
	unsigned int delay_flush;

	bool clear_fb;

	void __iomem *base;
	void __iomem *ipu_base;

	const struct jz_panel *panel;
};

static void *lcd_frame1;

static bool keep_aspect_ratio = true;
static bool allow_downscaling = false;

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

static int jzfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
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
static int jzfb_mmap(struct fb_info *fb, struct vm_area_struct *vma)
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

static int reduce_fraction(unsigned int *num, unsigned int *denom)
{
	unsigned long d = gcd(*num, *denom);

	/* The scaling table has only 31 entries */
	if (*num > 31 * d)
		return -EINVAL;

	*num /= d;
	*denom /= d;
	return 0;
}

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int jzfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	const struct jz_panel *panel = jzfb->panel;
	unsigned int num, denom;
	unsigned int framerate, divider;

	/* The minimum input size for the IPU to work is 4x4 */
	if (var->xres < 4)
		var->xres = 4;
	if (var->yres < 4)
		var->yres = 4;

	if (!allow_downscaling) {
		if (var->xres > panel->dw)
			var->xres = panel->dw;
		if (var->yres > panel->dh)
			var->yres = panel->dh;
	}

	/* Adjust the input size until we find a valid configuration */
	for (num = panel->dw, denom = var->xres; var->xres <= MAX_XRES &&
			reduce_fraction(&num, &denom) < 0;
			denom++, var->xres++);
	if (var->xres > MAX_XRES)
		return -EINVAL;

	for (num = panel->dh, denom = var->yres; var->yres <= MAX_YRES &&
			reduce_fraction(&num, &denom) < 0;
			denom++, var->yres++);
	if (var->yres > MAX_YRES)
		return -EINVAL;

	/* Reserve space for triple buffering. */
	var->yres_virtual = var->yres * 3;

	var->xres_virtual = var->xres;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->yoffset = 0;

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

	jzfb->clear_fb = var->bits_per_pixel != fb->var.bits_per_pixel ||
		var->xres != fb->var.xres || var->yres != fb->var.yres;

	divider = (panel->bw + panel->elw + panel->blw)
		* (panel->bh + panel->efw + panel->bfw);
	if (var->pixclock) {
		framerate = var->pixclock / divider;
		if (framerate > panel->fclk)
			framerate = panel->fclk;
	} else {
		framerate = panel->fclk;
	}

	var->pixclock = framerate * divider;
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
	writel((u32) jzfb->fb->fix.smem_start + jzfb->pan_offset,
			jzfb->ipu_base + IPU_Y_ADDR);
}

static void jzfb_lcdc_enable(struct jzfb *jzfb)
{
	jzfb_update_frame_address(jzfb);

	jzfb->delay_flush = 0;
	writel(0, jzfb->base + LCD_STATE); /* Clear LCDC status */

	/*
	 * Enabling the LCDC too soon after the clock will hang the system.
	 * A very short delay seems to be sufficient.
	 */
	udelay(1);

	ctrl_enable(jzfb);
}

static void jzfb_foreground_resize(struct jzfb *jzfb,
		unsigned int xpos, unsigned int ypos,
		unsigned int width, unsigned int height)
{
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

	writel((ypos << 16) | xpos, jzfb->base + LCD_XYP1);
	writel((height << 16) | width, jzfb->base + LCD_SIZE1);
}

static void jzfb_ipu_enable(struct jzfb *jzfb)
{
	u32 ctrl;

	/* Clear the status register and enable the chip */
	writel(0, jzfb->ipu_base + IPU_STATUS);

	ctrl = readl(jzfb->ipu_base + IPU_CTRL);
	writel(ctrl | IPU_CTRL_CHIP_EN | IPU_CTRL_RUN,
			jzfb->ipu_base + IPU_CTRL);
}

static void jzfb_ipu_disable(struct jzfb *jzfb)
{
	unsigned int timeout = 1000;
	u32 ctrl = readl(jzfb->ipu_base + IPU_CTRL);

	if (ctrl & IPU_CTRL_CHIP_EN) {
		writel(ctrl | IPU_CTRL_STOP, jzfb->ipu_base + IPU_CTRL);
		do {
			u32 status = readl(jzfb->ipu_base + IPU_STATUS);
			if (status & IPU_STATUS_OUT_END)
				break;
			msleep(1);
		} while (--timeout);

		if (!timeout)
			dev_err(&jzfb->pdev->dev,
					"Timeout while disabling IPU\n");
	}

	writel(ctrl & ~IPU_CTRL_CHIP_EN, jzfb->ipu_base + IPU_CTRL);
}

static void set_downscale_bilinear_coefs(struct jzfb *jzfb, unsigned int reg,
		unsigned int num, unsigned int denom)
{
	unsigned int i, weight_num = denom;

	for (i = 0; i < num; i++) {
		unsigned int weight, offset;
		u32 value;

		weight_num = num + (weight_num - num) % (num * 2);

		/*
		 * Here, "input pixel 1.0" means half of 0 and half of 1;
		 * "input pixel 0.5" means all of 0; and
		 * "input pixel 1.49" means almost all of 1.
		 */
		weight = 512 - 512 * (weight_num - num) / (num * 2);
		weight_num += denom * 2;
		offset = (weight_num - num) / (num * 2);

		value = ((weight & 0x7FF) << 6) | (offset << 1);
		writel(value, jzfb->ipu_base + reg);
	}
}

static void set_upscale_bilinear_coefs(struct jzfb *jzfb, unsigned int reg,
		unsigned int num, unsigned int denom)
{
	unsigned int i, weight_num = 0;

	for (i = 0; i < num; i++) {
		unsigned int weight = 512 - 512 * weight_num / num;
		u32 offset = 0, value;

		weight_num += denom;
		if (weight_num >= num) {
			weight_num -= num;
			offset = 1;
		}

		value = (weight & 0x7FF) << 6 | (offset << 1);
		writel(value, jzfb->ipu_base + reg);
	}
}

static void set_upscale_nearest_neighbour_coefs(struct jzfb *jzfb,
			unsigned int reg, unsigned int num)
{
	unsigned int i, weight_num = 1;

	for (i = 0; i < num; i++, weight_num++) {
		u32 value, offset = weight_num / num;
		weight_num %= num;

		value = (512 << 6) | (offset << 1);
		writel(value, jzfb->ipu_base + reg);
	}
}

static void set_coefs(struct jzfb *jzfb, unsigned int reg,
		unsigned int num, unsigned int denom)
{
	/* Start programmation of the LUT */
	writel(1, jzfb->ipu_base + reg);

	if (denom > num)
		set_downscale_bilinear_coefs(jzfb, reg, num, denom);
	else if (denom == 1)
		set_upscale_nearest_neighbour_coefs(jzfb, reg, num);
	else
		set_upscale_bilinear_coefs(jzfb, reg, num, denom);
}

static inline bool scaling_required(struct jzfb *jzfb)
{
	struct fb_var_screeninfo *var = &jzfb->fb->var;
	return var->xres != jzfb->panel->dw || var->yres != jzfb->panel->dh;
}

static void jzfb_ipu_configure(struct jzfb *jzfb)
{
	const struct jz_panel *panel = jzfb->panel;
	struct fb_info *fb = jzfb->fb;
	u32 ctrl, coef_index = 0, size, format = 2 << IPU_D_FMT_OUT_FMT_BIT;
	unsigned int outputW = panel->dw,
		     outputH = panel->dh,
		     xpos = 0, ypos = 0;

	/* Enable the chip, reset all the registers */
	writel(IPU_CTRL_CHIP_EN | IPU_CTRL_RST, jzfb->ipu_base + IPU_CTRL);

	switch (jzfb->bpp) {
	case 16:
		format |= 3 << IPU_D_FMT_IN_FMT_BIT;
		break;
	case 32:
	default:
		format |= 2 << IPU_D_FMT_IN_FMT_BIT;
		break;
	}
	writel(format, jzfb->ipu_base + IPU_D_FMT);

	/* Set the input height/width/stride */
	size = fb->fix.line_length << IPU_IN_GS_W_BIT
		| fb->var.yres << IPU_IN_GS_H_BIT;
	writel(size, jzfb->ipu_base + IPU_IN_GS);
	writel(fb->fix.line_length, jzfb->ipu_base + IPU_Y_STRIDE);

	/* Set the input address */
	writel((u32) fb->fix.smem_start, jzfb->ipu_base + IPU_Y_ADDR);

	ctrl = IPU_CTRL_CHIP_EN | IPU_CTRL_LCDC_SEL | IPU_CTRL_FM_IRQ_EN;
	if (fb->fix.type == FB_TYPE_PACKED_PIXELS)
		ctrl |= IPU_CTRL_SPKG_SEL;

	if (scaling_required(jzfb)) {
		unsigned int numW = panel->dw, denomW = fb->var.xres,
			     numH = panel->dh, denomH = fb->var.yres;

		BUG_ON(reduce_fraction(&numW, &denomW) < 0);
		BUG_ON(reduce_fraction(&numH, &denomH) < 0);

		if (keep_aspect_ratio) {
			unsigned int ratioW = (UINT_MAX >> 6) * numW / denomW,
				     ratioH = (UINT_MAX >> 6) * numH / denomH;
			if (ratioW < ratioH) {
				numH = numW;
				denomH = denomW;
			} else {
				numW = numH;
				denomW = denomH;
			}
		}

		if (numW != 1 || denomW != 1) {
			set_coefs(jzfb, IPU_HRSZ_COEF_LUT, numW, denomW);
			coef_index |= ((numW - 1) << 16);
			ctrl |= IPU_CTRL_HRSZ_EN;
		}

		if (numH != 1 || denomH != 1) {
			set_coefs(jzfb, IPU_VRSZ_COEF_LUT, numH, denomH);
			coef_index |= numH - 1;
			ctrl |= IPU_CTRL_VRSZ_EN;
		}

		outputH = fb->var.yres * numH / denomH;
		outputW = fb->var.xres * numW / denomW;

		/*
		 * If we are upscaling horizontally, the last columns of pixels
		 * shall be hidden, as they usually contain garbage: the last
		 * resizing coefficients, when applied to the last column of the
		 * input frame, instruct the IPU to blend the pixels with the
		 * ones that correspond to the next column, that is to say the
		 * leftmost column of pixels of the input frame.
		 */
		if (numW > denomW && denomW != 1)
			outputW -= numW / denomW;
	}

	writel(ctrl, jzfb->ipu_base + IPU_CTRL);

	/* Set the LUT index register */
	writel(coef_index, jzfb->ipu_base + IPU_RSZ_COEF_INDEX);

	/* Set the output height/width/stride */
	size = (outputW * 4) << IPU_OUT_GS_W_BIT
		| outputH << IPU_OUT_GS_H_BIT;
	writel(size, jzfb->ipu_base + IPU_OUT_GS);
	writel(outputW * 4, jzfb->ipu_base + IPU_OUT_STRIDE);

	/* Resize Foreground1 to the output size of the IPU */
	xpos = (panel->bw - outputW) / 2;
	ypos = (panel->bh - outputH) / 2;
	jzfb_foreground_resize(jzfb, xpos, ypos, outputW, outputH);

	dev_dbg(&jzfb->pdev->dev, "Scaling %ux%u to %ux%u\n",
			fb->var.xres, fb->var.yres, outputW, outputH);
}

static void jzfb_power_up(struct jzfb *jzfb)
{
	pinctrl_pm_select_default_state(&jzfb->pdev->dev);

	clk_enable(jzfb->lpclk);
	jzfb_lcdc_enable(jzfb);

	clk_enable(jzfb->ipuclk);
	jzfb_ipu_enable(jzfb);
}

static void jzfb_power_down(struct jzfb *jzfb)
{
	ctrl_disable(jzfb);
	clk_disable(jzfb->lpclk);

	jzfb_ipu_disable(jzfb);
	clk_disable(jzfb->ipuclk);

	pinctrl_pm_select_sleep_state(&jzfb->pdev->dev);
}

/*
 * (Un)blank the display.
 */
static int jzfb_blank(int blank_mode, struct fb_info *info)
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

static int jzfb_pan_display(struct fb_var_screeninfo *var,
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
static int jzfb_map_smem(struct fb_info *fb)
{
	/* Compute space for max res at 32bpp, triple buffered. */
	unsigned int size = PAGE_ALIGN(MAX_XRES * MAX_YRES * 4 * 3);
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

static void jzfb_unmap_smem(struct fb_info *fb)
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

static void jzfb_set_panel_mode(struct jzfb *jzfb)
{
	const struct jz_panel *panel = jzfb->panel;

	/* Configure LCDC */
	writel(panel->cfg, jzfb->base + LCD_CFG);

	/* Enable IPU auto-restart */
	writel(LCD_IPUR_IPUREN |
			(panel->blw + panel->bw + panel->elw) * panel->vsw / 3,
			jzfb->base + LCD_IPUR);

	/* Set HT / VT / HDS / HDE / VDS / VDE / HPE / VPE */
	writel((panel->blw + panel->bw + panel->elw) << LCD_VAT_HT_BIT |
			(panel->bfw + panel->bh + panel->efw) << LCD_VAT_VT_BIT,
		jzfb->base + LCD_VAT);
	writel(panel->blw << LCD_DAH_HDS_BIT |
			(panel->blw + panel->bw) << LCD_DAH_HDE_BIT,
			jzfb->base + LCD_DAH);
	writel(panel->bfw << LCD_DAV_VDS_BIT |
			(panel->bfw + panel->bh) << LCD_DAV_VDE_BIT,
			jzfb->base + LCD_DAV);
	writel(panel->hsw << LCD_HSYNC_HPE_BIT, jzfb->base + LCD_HSYNC);
	writel(panel->vsw << LCD_VSYNC_VPE_BIT, jzfb->base + LCD_VSYNC);

	/* Enable foreground 1, OSD mode */
	writew(LCD_OSDC_F1EN | LCD_OSDC_OSDEN, jzfb->base + LCD_OSDC);

	/* Enable IPU, 18/24 bpp output */
	writew(LCD_OSDCTRL_IPU | LCD_OSDCTRL_OSDBPP_18_24,
			jzfb->base + LCD_OSDCTRL);

	/* Set a black background */
	writel(0, jzfb->base + LCD_BGC);
}

static void jzfb_change_clock(struct jzfb *jzfb, unsigned int rate)
{
	/* Use pixel clock for LCD panel (as opposed to TV encoder). */
	__cpm_select_pixclk_lcd();

	rate = clk_round_rate(jzfb->lpclk, rate);
	clk_set_rate(jzfb->lpclk, rate);

	dev_dbg(&jzfb->pdev->dev, "PixClock: req %u, got %lu\n",
		rate, clk_get_rate(jzfb->lpclk));
}

/* set the video mode according to info->var */
static int jzfb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct jzfb *jzfb = info->par;

	if (jzfb->is_enabled) {
		ctrl_disable(jzfb);
		jzfb_ipu_disable(jzfb);
	} else {
		clk_enable(jzfb->lpclk);
		clk_enable(jzfb->ipuclk);
	}

	jzfb->pan_offset = 0;
	jzfb->bpp = var->bits_per_pixel;
	fix->line_length = var->xres_virtual * (var->bits_per_pixel >> 3);

	jzfb_set_panel_mode(jzfb);
	jzfb_ipu_configure(jzfb);

	/* Clear the framebuffer to avoid artifacts */
	if (jzfb->clear_fb) {
		void *page_virt = lcd_frame1;
		unsigned int size = fix->line_length * var->yres * 3;

		for (; page_virt < lcd_frame1 + size; page_virt += PAGE_SIZE)
			clear_page(page_virt);
		dma_cache_wback_inv((unsigned long) lcd_frame1, size);
	}

	if (jzfb->is_enabled) {
		jzfb_ipu_enable(jzfb);
		jzfb_lcdc_enable(jzfb);
	} else {
		clk_disable(jzfb->lpclk);
		clk_disable(jzfb->ipuclk);
	}

	fix->visual = FB_VISUAL_TRUECOLOR;
	return 0;
}

static void jzfb_ipu_reset(struct jzfb *jzfb)
{
	ctrl_disable(jzfb);
	clk_enable(jzfb->ipuclk);
	jzfb_ipu_disable(jzfb);
	writel(IPU_CTRL_CHIP_EN | IPU_CTRL_RST, jzfb->ipu_base + IPU_CTRL);

	jzfb_set_panel_mode(jzfb);
	jzfb_ipu_configure(jzfb);
	jzfb_ipu_enable(jzfb);
	ctrl_enable(jzfb);
}

static int jzfb_ioctl(struct fb_info *info, unsigned int cmd,
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

static struct fb_ops jzfb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= jzfb_setcolreg,
	.fb_check_var		= jzfb_check_var,
	.fb_set_par		= jzfb_set_par,
	.fb_blank		= jzfb_blank,
	.fb_pan_display		= jzfb_pan_display,
	.fb_fillrect		= sys_fillrect,
	.fb_copyarea		= sys_copyarea,
	.fb_imageblit		= sys_imageblit,
	.fb_mmap		= jzfb_mmap,
	.fb_ioctl		= jzfb_ioctl,
};

static irqreturn_t jzfb_interrupt_handler(int irq, void *dev_id)
{
	struct jzfb *jzfb = dev_id;

	if (jzfb->delay_flush == 0) {
		struct fb_info *fb = jzfb->fb;
		dma_cache_wback_inv((unsigned long)(lcd_frame1 +
					jzfb->pan_offset),
				fb->fix.line_length * fb->var.yres);
	} else {
		jzfb->delay_flush--;
	}

	jzfb_update_frame_address(jzfb);
	jzfb->vsync_count++;
	wake_up_interruptible_all(&jzfb->wait_vsync);

	writel(0, jzfb->ipu_base + IPU_STATUS);
	return IRQ_HANDLED;
}

static ssize_t keep_aspect_ratio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%c\n", keep_aspect_ratio ? 'Y' : 'N');
}

static ssize_t keep_aspect_ratio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	bool new_value = false;

	if (strtobool(buf, &new_value) < 0)
		return -EINVAL;

	keep_aspect_ratio = new_value;

	if (jzfb->is_enabled && scaling_required(jzfb)) {
		ctrl_disable(jzfb);
		jzfb_ipu_disable(jzfb);
		jzfb_ipu_configure(jzfb);
		jzfb_ipu_enable(jzfb);
		jzfb_lcdc_enable(jzfb);
	}

	return count;
}

static DEVICE_ATTR_RW(keep_aspect_ratio);
static DEVICE_BOOL_ATTR(allow_downscaling, 0644, allow_downscaling);

static int jzfb_probe(struct platform_device *pdev)
{
	struct jzfb *jzfb;
	struct fb_info *fb;
	struct resource *res;
	int ret;

	fb = framebuffer_alloc(sizeof(struct jzfb), &pdev->dev);
	if (!fb) {
		dev_err(&pdev->dev, "Failed to allocate framebuffer device\n");
		return -ENOMEM;
	}

	jzfb = fb->par;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	jzfb->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jzfb->base)) {
		dev_err(&pdev->dev, "Failed to request LCD registers\n");
		ret = PTR_ERR(jzfb->base);
		goto err_release_fb;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	jzfb->ipu_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jzfb->ipu_base)) {
		dev_err(&pdev->dev, "Failed to request IPU registers\n");
		ret = PTR_ERR(jzfb->ipu_base);
		goto err_release_fb;
	}

	jzfb->panel = &jz4770_lcd_panel;
	jzfb->pdev = pdev;
	jzfb->bpp = 32;
	init_waitqueue_head(&jzfb->wait_vsync);

	strcpy(fb->fix.id, "jz-lcd");
	fb->fix.type	= FB_TYPE_PACKED_PIXELS;
	fb->fix.type_aux	= 0;

	fb->fix.xpanstep	= 1;
	fb->fix.ypanstep	= 1;
	fb->fix.ywrapstep	= 0;
	fb->fix.accel	= FB_ACCEL_NONE;
	fb->fix.visual = FB_VISUAL_TRUECOLOR;

	fb->var.nonstd	= 0;
	fb->var.activate	= FB_ACTIVATE_NOW;
	fb->var.height	= -1;
	fb->var.width	= -1;
	fb->var.accel_flags	= FB_ACCELF_TEXT;
	fb->var.bits_per_pixel = jzfb->bpp;

	fb->var.xres = jzfb->panel->dw;
	fb->var.yres = jzfb->panel->dh;
	fb->var.vmode = FB_VMODE_NONINTERLACED;

	jzfb_check_var(&fb->var, fb);

	fb->fbops		= &jzfb_ops;
	fb->flags		= FBINFO_FLAG_DEFAULT;

	fb->pseudo_palette	= jzfb->pseudo_palette;
	INIT_LIST_HEAD(&fb->modelist);

	ret = jzfb_map_smem(fb);
	if (ret)
		goto err_release_fb;

	/* Init pixel clock. */
	jzfb->lpclk = devm_clk_get(&pdev->dev, "lpclk");
	if (IS_ERR(jzfb->lpclk)) {
		ret = PTR_ERR(jzfb->lpclk);
		dev_err(&pdev->dev, "Failed to get pixel clock: %d\n", ret);
		goto err_unmap;
	}

	jzfb->ipuclk = devm_clk_get(&pdev->dev, "ipu");
	if (IS_ERR(jzfb->ipuclk)) {
		ret = PTR_ERR(jzfb->ipuclk);
		dev_err(&pdev->dev, "Failed to get ipu clock: %d\n", ret);
		goto err_unmap;
	}

	if (request_irq(IRQ_IPU, jzfb_interrupt_handler, 0,
				"ipu", jzfb)) {
		dev_err(&pdev->dev, "Failed to request IRQ.\n");
		ret = -EBUSY;
		goto err_unmap;
	}

	mutex_init(&jzfb->lock);

	platform_set_drvdata(pdev, jzfb);
	jzfb->fb = fb;

	/*
	 * We assume the LCDC is disabled initially. If you really must have
	 * video in your boot loader, you'll have to update this driver.
	 */
	ret = clk_prepare(jzfb->ipuclk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to prepare IPU clock: %i\n", ret);
		goto err_unmap;
	}

	ret = clk_prepare(jzfb->lpclk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to prepare pixel clock: %i\n", ret);
		goto err_unprepare_ipuclk;
	}

	jzfb_change_clock(jzfb, fb->var.pixclock);
	clk_enable(jzfb->lpclk);

	fb->fix.line_length = fb->var.xres_virtual * (fb->var.bits_per_pixel >> 3);

	jzfb->delay_flush = 0;

	jzfb_ipu_reset(jzfb);
	jzfb->is_enabled = true;

	ret = device_create_file(&pdev->dev, &dev_attr_keep_aspect_ratio);
	if (ret) {
		dev_err(&pdev->dev, "Unable to create sysfs node: %i\n", ret);
		goto err_unprepare_lpclk;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_allow_downscaling.attr);
	if (ret) {
		dev_err(&pdev->dev, "Unable to create sysfs node: %i\n", ret);
		goto err_remove_keep_aspect_ratio_file;
	}

	ret = register_framebuffer(fb);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer device.\n");
		goto err_remove_allow_downscaling_file;
	}
	dev_info(&pdev->dev,
		"fb%d: %s frame buffer device, using %dK of video memory\n",
		fb->node, fb->fix.id, fb->fix.smem_len>>10);

	fb_prepare_logo(jzfb->fb, 0);
	fb_show_logo(jzfb->fb, 0);
	return 0;

err_remove_allow_downscaling_file:
	device_remove_file(&pdev->dev, &dev_attr_allow_downscaling.attr);
err_remove_keep_aspect_ratio_file:
	device_remove_file(&pdev->dev, &dev_attr_keep_aspect_ratio);
err_unprepare_lpclk:
	clk_disable_unprepare(jzfb->lpclk);
err_unprepare_ipuclk:
	clk_unprepare(jzfb->ipuclk);
err_unmap:
	jzfb_unmap_smem(fb);
err_release_fb:
	framebuffer_release(fb);
	return ret;
}

static int jzfb_remove(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_allow_downscaling.attr);
	device_remove_file(&pdev->dev, &dev_attr_keep_aspect_ratio);

	if (jzfb->is_enabled)
		jzfb_power_down(jzfb);

	clk_unprepare(jzfb->lpclk);
	clk_unprepare(jzfb->ipuclk);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int jzfb_suspend(struct device *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	dev_dbg(dev, "Suspending\n");

	if (jzfb->is_enabled)
		jzfb_power_down(jzfb);

	return 0;
}

static int jzfb_resume(struct device *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	dev_dbg(dev, "Resuming\n");

	if (jzfb->is_enabled)
		jzfb_power_up(jzfb);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(jzfb_pm_ops, jzfb_suspend, jzfb_resume);

static struct platform_driver jzfb_driver = {
	.probe	= jzfb_probe,
	.remove = jzfb_remove,
	.driver = {
		.name  = "jz-lcd",
		.pm    = &jzfb_pm_ops,
	},
};

module_platform_driver(jzfb_driver);

MODULE_DESCRIPTION("Jz4770 LCD frame buffer driver");
MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_LICENSE("GPL");

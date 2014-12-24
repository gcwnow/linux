/*
 * linux/drivers/remoteproc/jz4770_remoteproc.c
 *
 * Remoteproc driver to manage VPU for JZ4770.
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 * Copyright (C) 2013  Wladimir J. van der Laan
 * Copyright (C) 2013  Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2014  Paul Cercueil <paul@crapouillou.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>

#include "remoteproc_internal.h"

#define VPU_DEFAULT_FW_NAME "jz4770_vpu_fw.elf"

#define REG_AUX_CTRL		0x0
#define REG_AUX_MSG_ACK		0x10
#define REG_AUX_MSG		0x14
#define REG_CORE_MSG_ACK	0x18
#define REG_CORE_MSG		0x1C

#define AUX_CTRL_SLEEP		BIT(31)
#define AUX_CTRL_MSG_IRQ_EN	BIT(3)
#define AUX_CTRL_IRQ_MODE	BIT(2)
#define AUX_CTRL_IRQ		BIT(1)
#define AUX_CTRL_SW_RESET	BIT(0)


/* Device data */
struct vpu {
	int irq;
	struct clk *vpu_clk;
	struct clk *aux_clk;
	void __iomem *aux_base;
	void __iomem *sch_base;
	struct device *dev;
};

static char *vpu_fw_name = VPU_DEFAULT_FW_NAME;
module_param(vpu_fw_name, charp, S_IRUGO);
MODULE_PARM_DESC(vpu_fw_name,
		 "\n\t\tName of VPU firmware file in /lib/firmware"
		 " (if not specified defaults to '" VPU_DEFAULT_FW_NAME "')");

static int jz4770_rproc_start(struct rproc *rproc)
{
	struct vpu *vpu = rproc->priv;
	u32 ctrl;

	clk_enable(vpu->vpu_clk);
	clk_enable(vpu->aux_clk);
	enable_irq(vpu->irq);

	/* Disable TLB (for now) */
	writel(BIT(9), vpu->sch_base + 0x0);

	/* Clear the reset bit, enable IRQ to AUX, disable reset on IRQ */
	ctrl = AUX_CTRL_IRQ_MODE | AUX_CTRL_MSG_IRQ_EN;
	writel(ctrl, vpu->aux_base + REG_AUX_CTRL);
	return 0;
}

static int jz4770_rproc_stop(struct rproc *rproc)
{
	struct vpu *vpu = rproc->priv;

	/* Keep AUX in reset mode */
	writel(AUX_CTRL_SW_RESET, vpu->aux_base + REG_AUX_CTRL);

	disable_irq_nosync(vpu->irq);
	clk_disable(vpu->aux_clk);
	clk_disable(vpu->vpu_clk);
	return 0;
}

static void jz4770_rproc_kick(struct rproc *rproc, int vqid)
{
	struct vpu *vpu = rproc->priv;
	writel(vqid, vpu->aux_base + REG_CORE_MSG);
}

static struct rproc_ops jz4770_rproc_ops = {
	.start = jz4770_rproc_start,
	.stop = jz4770_rproc_stop,
	.kick = jz4770_rproc_kick,
};

static irqreturn_t vpu_interrupt(int irq, void *data)
{
	struct vpu *vpu = data;
	struct rproc *rproc = dev_get_drvdata(vpu->dev);
	u32 vring = readl(vpu->aux_base + REG_AUX_MSG);

	writel(0, vpu->aux_base + REG_CORE_MSG_ACK);

	dev_dbg(vpu->dev, "Received message for vring %u\n", vring);
	rproc_vq_interrupt(rproc, vring);
	return IRQ_HANDLED;
}

static int jz4770_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vpu *vpu;
	void __iomem *aux_base;
	void __iomem *sch_base;
	struct clk *vpu_clk, *aux_clk;
	int ret, irq;
	struct rproc *rproc;

	aux_base = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(aux_base)) {
		ret = PTR_ERR(aux_base);
		dev_err(dev, "Failed to get and remap mmio region: %d\n", ret);
		return ret;
	}

	sch_base = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 1));
	if (IS_ERR(sch_base)) {
		ret = PTR_ERR(sch_base);
		dev_err(dev, "Failed to get and remap mmio region: %d\n", ret);
		return ret;
	}

	/*
	 * Note: TCSM0 is also declared as a platform resource, but we do not
	 *       have any driver code yet that accesses it.
	 */

	vpu_clk = devm_clk_get(dev, "vpu");
	if (IS_ERR(vpu_clk)) {
		ret = PTR_ERR(vpu_clk);
		dev_err(dev, "Failed to get VPU clock: %d\n", ret);
		return ret;
	}

	aux_clk = devm_clk_get(dev, "aux");
	if (IS_ERR(aux_clk)) {
		ret = PTR_ERR(aux_clk);
		dev_err(dev, "Failed to get AUX clock: %d\n", ret);
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Failed to get platform IRQ: %d\n", irq);
		return irq;
	}

	rproc = rproc_alloc(dev, "jz4770-vpu", &jz4770_rproc_ops,
			vpu_fw_name, sizeof(*vpu));
	if (!rproc)
		return -ENOMEM;

	platform_set_drvdata(pdev, rproc);
	vpu = rproc->priv;
	vpu->dev = &pdev->dev;

	vpu->aux_base = aux_base;
	vpu->sch_base = sch_base;
	vpu->vpu_clk = vpu_clk;
	vpu->aux_clk = aux_clk;
	vpu->irq = irq;

	ret = devm_request_irq(dev, irq, vpu_interrupt, 0, "jz4770-vpu", vpu);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		goto free_rproc;
	}

	disable_irq_nosync(irq);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "Failed to register remote processor: %d\n", ret);
		goto free_rproc;
	}

	return 0;

free_rproc:
	rproc_put(rproc);
	return ret;
}

static int jz4770_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_shutdown(rproc);
	rproc_del(rproc);
	rproc_put(rproc);
	return 0;
}

static struct platform_driver jz4770_rproc_driver = {
	.probe = jz4770_rproc_probe,
	.remove = jz4770_rproc_remove,
	.driver = {
		.name = "jz4770-vpu",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(jz4770_rproc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ4770 Remote Processor control driver");

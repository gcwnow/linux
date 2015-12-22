/*
 *  Copyright (C) 2010, Paul Cercueil <paul@crapouillou.net>
 *  JZ4740 Watchdog driver
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/ingenic-tcu.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define DEFAULT_HEARTBEAT 5
#define MAX_HEARTBEAT     2048

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int heartbeat = DEFAULT_HEARTBEAT;
module_param(heartbeat, uint, 0);
MODULE_PARM_DESC(heartbeat,
		"Watchdog heartbeat period in seconds from 1 to "
		__MODULE_STRING(MAX_HEARTBEAT) ", default "
		__MODULE_STRING(DEFAULT_HEARTBEAT));

struct jz4740_wdt_drvdata {
	struct watchdog_device wdt;
	struct regmap *map;
	struct clk *clk;
};

static int jz4740_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);

	regmap_write(drvdata->map, TCU_REG_WDT_TCNT, 0);
	return 0;
}

static int jz4740_wdt_set_timeout(struct watchdog_device *wdt_dev,
				    unsigned int new_timeout)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);
	struct clk *clk = drvdata->clk;
	unsigned long clk_rate, timeout_value;
	bool change_rate;
	u32 tcer;
	int ret = 0;

	clk_rate = clk_get_rate(clk);
	timeout_value = clk_rate * new_timeout;
	change_rate = false;

	if (timeout_value > 0xffff) {
		clk_rate = clk_round_rate(clk, 0xffff / new_timeout);
		timeout_value = clk_rate * new_timeout;
		if (timeout_value > 0xffff)
			return -EINVAL;
		change_rate = true;
	}

	regmap_read(drvdata->map, TCU_REG_WDT_TCER, &tcer);
	regmap_write(drvdata->map, TCU_REG_WDT_TCER, 0);

	if (change_rate) {
		clk_disable_unprepare(clk);
		ret = clk_set_rate(clk, clk_rate);
		clk_prepare_enable(clk);
		if (ret)
			goto done;
	}

	regmap_write(drvdata->map, TCU_REG_WDT_TDR, (u16)timeout_value);
	regmap_write(drvdata->map, TCU_REG_WDT_TCNT, 0);

	wdt_dev->timeout = new_timeout;

done:
	regmap_write(drvdata->map, TCU_REG_WDT_TCER, tcer & BIT(0));
	return ret;
}

static int jz4740_wdt_start(struct watchdog_device *wdt_dev)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);
	int ret;

	clk_prepare_enable(drvdata->clk);
	ret = jz4740_wdt_set_timeout(wdt_dev, wdt_dev->timeout);
	if (ret)
		clk_disable_unprepare(drvdata->clk);
	else
		regmap_write(drvdata->map, TCU_REG_WDT_TCER, 1);

	return ret;
}

static int jz4740_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct jz4740_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);

	regmap_write(drvdata->map, TCU_REG_WDT_TCER, 0);
	clk_disable_unprepare(drvdata->clk);

	return 0;
}

static int jz4740_wdt_restart(struct watchdog_device *wdt_dev,
			      unsigned long action, void *data)
{
	wdt_dev->timeout = 0;
	jz4740_wdt_start(wdt_dev);
	return 0;
}

static const struct watchdog_info jz4740_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "jz4740 Watchdog",
};

static const struct watchdog_ops jz4740_wdt_ops = {
	.owner = THIS_MODULE,
	.start = jz4740_wdt_start,
	.stop = jz4740_wdt_stop,
	.ping = jz4740_wdt_ping,
	.set_timeout = jz4740_wdt_set_timeout,
	.restart = jz4740_wdt_restart,
};

#ifdef CONFIG_OF
static const struct of_device_id jz4740_wdt_of_matches[] = {
	{ .compatible = "ingenic,jz4740-watchdog", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jz4740_wdt_of_matches);
#endif

static int jz4740_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jz4740_wdt_drvdata *drvdata;
	struct watchdog_device *jz4740_wdt;
	int ret;

	if (!dev->of_node) {
		dev_err(dev, "jz4740-wdt must be probed via devicetree\n");
		return -ENODEV;
	}

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, drvdata);

	if (heartbeat < 1 || heartbeat > MAX_HEARTBEAT)
		heartbeat = DEFAULT_HEARTBEAT;

	jz4740_wdt = &drvdata->wdt;
	jz4740_wdt->info = &jz4740_wdt_info;
	jz4740_wdt->ops = &jz4740_wdt_ops;
	jz4740_wdt->timeout = heartbeat;
	jz4740_wdt->min_timeout = 1;
	jz4740_wdt->max_timeout = MAX_HEARTBEAT;
	jz4740_wdt->parent = dev;
	watchdog_set_nowayout(jz4740_wdt, nowayout);
	watchdog_set_drvdata(jz4740_wdt, drvdata);

	drvdata->map = syscon_node_to_regmap(dev->of_node->parent);
	if (IS_ERR(drvdata->map))
		return PTR_ERR(drvdata->map);

	drvdata->clk = devm_clk_get(&pdev->dev, "wdt");
	if (IS_ERR(drvdata->clk)) {
		dev_err(&pdev->dev, "cannot find WDT clock\n");
		return PTR_ERR(drvdata->clk);
	}

	ret = devm_watchdog_register_device(&pdev->dev, &drvdata->wdt);
	if (ret < 0)
		return ret;

	return 0;
}

static struct platform_driver jz4740_wdt_driver = {
	.probe = jz4740_wdt_probe,
	.driver = {
		.name = "jz4740-wdt",
		.of_match_table = of_match_ptr(jz4740_wdt_of_matches),
	},
};

module_platform_driver(jz4740_wdt_driver);

MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_DESCRIPTION("jz4740 Watchdog Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz4740-wdt");

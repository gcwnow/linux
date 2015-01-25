/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2014, Paul Cercueil <paul@crapouillou.net>
 *  JZ4740 platform PWM support
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define NUM_PWM 8

#define TCU_TSR_OFFSET	0x0C /* Timer Stop register */
#define TCU_TSSR_OFFSET	0x1C
#define TCU_TSCR_OFFSET	0x2C

#define TCU_TER_OFFSET	0x00 /* Timer Counter Enable register */
#define TCU_TESR_OFFSET	0x04
#define TCU_TECR_OFFSET	0x08

#define TCU_TDFR_OFFSET(pwm)	(0x30 + (pwm) * 0x10) /* Timer Data Full reg */
#define TCU_TDHR_OFFSET(pwm)	(0x34 + (pwm) * 0x10) /* Timer Data Half reg */
#define TCU_TCNT_OFFSET(pwm)	(0x38 + (pwm) * 0x10) /* Timer Counter reg */
#define TCU_TCSR_OFFSET(pwm)	(0x3C + (pwm) * 0x10) /* Timer Control reg */

#define TCU_TCSR_PWM_SD		BIT(9)  /* 0: Shutdown abruptly 1: gracefully */
#define TCU_TCSR_PWM_INITL_HIGH	BIT(8)  /* Sets the initial output level */
#define TCU_TCSR_PWM_EN		BIT(7)  /* PWM pin output enable */
#define TCU_TCSR_PRESCALE_MASK	0x0038
#define TCU_TCSR_PRESCALE_SHIFT	3

struct jz4740_pwm_chip {
	struct pwm_chip chip;
	void __iomem *base;
	struct clk *clk;
};

static inline struct jz4740_pwm_chip *to_jz4740(struct pwm_chip *chip)
{
	return container_of(chip, struct jz4740_pwm_chip, chip);
}

static int jz4740_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);

	/* Start clock */
	writel(BIT(pwm->hwpwm), jz->base + TCU_TSCR_OFFSET);
	return 0;
}

static void jz4740_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);

	/* Stop clock */
	writel(BIT(pwm->hwpwm), jz->base + TCU_TSSR_OFFSET);
}

static int jz4740_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);
	u16 tcsr;

	/* Enable PWM output */
	tcsr = readw(jz->base + TCU_TCSR_OFFSET(pwm->hwpwm));
	writew(tcsr | TCU_TCSR_PWM_EN, jz->base + TCU_TCSR_OFFSET(pwm->hwpwm));

	/* Start counter */
	writew(BIT(pwm->hwpwm), jz->base + TCU_TESR_OFFSET);
	return 0;
}

static void jz4740_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);
	u16 tcsr;

	/* Stop counter */
	writew(BIT(pwm->hwpwm), jz->base + TCU_TECR_OFFSET);

	/* Disable PWM output */
	tcsr = readw(jz->base + TCU_TCSR_OFFSET(pwm->hwpwm));
	writew(tcsr & ~TCU_TCSR_PWM_EN, jz->base + TCU_TCSR_OFFSET(pwm->hwpwm));
}

static bool tcu_counter_enabled(struct jz4740_pwm_chip *jz, unsigned int pwm)
{
	return readw(jz->base + TCU_TER_OFFSET) & BIT(pwm);
}

static int jz4740_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	struct jz4740_pwm_chip *jz4740 = to_jz4740(pwm->chip);
	unsigned long long tmp;
	unsigned long period, duty;
	unsigned int prescaler = 0;
	bool is_enabled;
	u16 tcsr;

	tmp = (unsigned long long)clk_get_rate(jz4740->clk) * period_ns;
	do_div(tmp, 1000000000);
	period = tmp;

	while (period > 0xffff && prescaler < 6) {
		period >>= 2;
		++prescaler;
	}

	if (prescaler == 6)
		return -EINVAL;

	tmp = (unsigned long long)period * duty_ns;
	do_div(tmp, period_ns);
	duty = tmp;

	if (duty >= period)
		duty = period - 1;

	is_enabled = tcu_counter_enabled(jz4740, pwm->hwpwm);
	if (is_enabled)
		jz4740_pwm_disable(chip, pwm);

	/* Set abrupt shutdown, clock divider */
	tcsr = readw(jz4740->base + TCU_TCSR_OFFSET(pwm->hwpwm));
	tcsr &= ~TCU_TCSR_PRESCALE_MASK;
	tcsr |= TCU_TCSR_PWM_SD | (prescaler << TCU_TCSR_PRESCALE_SHIFT);
	writew(tcsr, jz4740->base + TCU_TCSR_OFFSET(pwm->hwpwm));

	/* Reset counter to 0 */
	writew(0, jz4740->base + TCU_TCNT_OFFSET(pwm->hwpwm));

	/* Set duty */
	writew(duty, jz4740->base + TCU_TDHR_OFFSET(pwm->hwpwm));

	/* Set period */
	writew(period, jz4740->base + TCU_TDFR_OFFSET(pwm->hwpwm));

	if (is_enabled)
		jz4740_pwm_enable(chip, pwm);

	return 0;
}

static int jz4740_pwm_set_polarity(struct pwm_chip *chip,
		struct pwm_device *pwm, enum pwm_polarity polarity)
{
	struct jz4740_pwm_chip *jz = to_jz4740(chip);
	u16 reg = readw(jz->base + TCU_TCSR_OFFSET(pwm->hwpwm));

	switch (polarity) {
	case PWM_POLARITY_NORMAL:
		reg &= ~TCU_TCSR_PWM_INITL_HIGH;
		break;
	case PWM_POLARITY_INVERSED:
		reg |= TCU_TCSR_PWM_INITL_HIGH;
		break;
	}

	writew(reg, jz->base + TCU_TCSR_OFFSET(pwm->hwpwm));
	return 0;
}

static const struct pwm_ops jz4740_pwm_ops = {
	.request = jz4740_pwm_request,
	.free = jz4740_pwm_free,
	.config = jz4740_pwm_config,
	.set_polarity = jz4740_pwm_set_polarity,
	.enable = jz4740_pwm_enable,
	.disable = jz4740_pwm_disable,
	.owner = THIS_MODULE,
};

static int jz4740_pwm_probe(struct platform_device *pdev)
{
	struct jz4740_pwm_chip *jz4740;
	struct resource *res;

	jz4740 = devm_kzalloc(&pdev->dev, sizeof(*jz4740), GFP_KERNEL);
	if (!jz4740)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	jz4740->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jz4740->base))
		return PTR_ERR(jz4740->base);

	jz4740->clk = devm_clk_get(&pdev->dev, "ext");
	if (IS_ERR(jz4740->clk))
		return PTR_ERR(jz4740->clk);

	jz4740->chip.dev = &pdev->dev;
	jz4740->chip.ops = &jz4740_pwm_ops;
	jz4740->chip.npwm = NUM_PWM;
	jz4740->chip.base = -1;

	if (IS_ENABLED(CONFIG_OF) && pdev->dev.of_node) {
		jz4740->chip.of_xlate = of_pwm_xlate_with_flags;
		jz4740->chip.of_pwm_n_cells = 3;
	}

	platform_set_drvdata(pdev, jz4740);

	return pwmchip_add(&jz4740->chip);
}

static int jz4740_pwm_remove(struct platform_device *pdev)
{
	struct jz4740_pwm_chip *jz4740 = platform_get_drvdata(pdev);

	return pwmchip_remove(&jz4740->chip);
}

static const struct of_device_id jz4740_pwm_dt_ids[] = {
	{ .compatible = "ingenic,jz4740-pwm", .data = NULL, },
	{ .compatible = "ingenic,jz4770-pwm", .data = NULL, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jz4740_pwm_dt_ids);

static struct platform_driver jz4740_pwm_driver = {
	.driver = {
		.name = "jz4740-pwm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(jz4740_pwm_dt_ids),
	},
	.probe = jz4740_pwm_probe,
	.remove = jz4740_pwm_remove,
};
module_platform_driver(jz4740_pwm_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Ingenic JZ4740 PWM driver");
MODULE_ALIAS("platform:jz4740-pwm");
MODULE_LICENSE("GPL");

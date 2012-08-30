/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2012, Paul Cercueil <paul@crapouillou.net>
 *  JZ4770 platform PWM support
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
#include <linux/platform_device.h>
#include <linux/pwm.h>

#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/jz4770tcu.h>

#define NUM_PWM 8

static const unsigned int jz4770_pwm_gpio_list[NUM_PWM] = {
	GPE(0),
	GPE(1),
	GPE(2),
	GPE(3),
	GPE(4),
	GPE(5),
	GPD(10),
	GPD(11),
};

struct jz4770_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
};

static inline struct jz4770_pwm_chip *to_jz4770(struct pwm_chip *chip)
{
	return container_of(chip, struct jz4770_pwm_chip, chip);
}

static int jz4770_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int gpio = jz4770_pwm_gpio_list[pwm->hwpwm];
	int ret;

	ret = gpio_request(gpio, pwm->label);
	if (ret) {
		dev_err(chip->dev, "Failed to request GPIO#%u for PWM: %d\n",
			gpio, ret);
		return ret;
	}

	__gpio_as_func0(gpio);

	__tcu_start_timer_clock(pwm->hwpwm);

	return 0;
}

static void jz4770_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int gpio = jz4770_pwm_gpio_list[pwm->hwpwm];

	__tcu_stop_timer_clock(pwm->hwpwm);
	__gpio_as_input(gpio);
	gpio_free(gpio);
}

static int jz4770_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	__tcu_enable_pwm_output(pwm->hwpwm);
	__tcu_start_counter(pwm->hwpwm);
	return 0;
}

static void jz4770_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	__tcu_stop_counter(pwm->hwpwm);
	__tcu_disable_pwm_output(pwm->hwpwm);
}

static int jz4770_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	struct jz4770_pwm_chip *jz4770 = to_jz4770(pwm->chip);
	unsigned long long tmp;
	unsigned long period, duty;
	unsigned int prescaler = 0;
	bool is_enabled;

	tmp = (unsigned long long)clk_get_rate(jz4770->clk) * period_ns;
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

	is_enabled = __tcu_counter_enabled(pwm->hwpwm);
	if (is_enabled)
		jz4770_pwm_disable(chip, pwm);

	__tcu_init_pwm_output_high(pwm->hwpwm);
	__tcu_set_count(pwm->hwpwm, 0);
	__tcu_set_half_data(pwm->hwpwm, duty);
	__tcu_set_full_data(pwm->hwpwm, period);
	__tcu_select_extalclk(pwm->hwpwm);
	__tcu_select_clk_div(pwm->hwpwm, prescaler);
	__tcu_set_pwm_output_shutdown_abrupt(pwm->hwpwm);

	if (is_enabled)
		jz4770_pwm_enable(chip, pwm);

	return 0;
}

static const struct pwm_ops jz4770_pwm_ops = {
	.request = jz4770_pwm_request,
	.free = jz4770_pwm_free,
	.config = jz4770_pwm_config,
	.enable = jz4770_pwm_enable,
	.disable = jz4770_pwm_disable,
	.owner = THIS_MODULE,
};

static int __devinit jz4770_pwm_probe(struct platform_device *pdev)
{
	struct jz4770_pwm_chip *jz4770;
	int ret;

	jz4770 = devm_kzalloc(&pdev->dev, sizeof(*jz4770), GFP_KERNEL);
	if (!jz4770)
		return -ENOMEM;

	jz4770->clk = clk_get(NULL, "ext");
	if (IS_ERR(jz4770->clk))
		return PTR_ERR(jz4770->clk);

	jz4770->chip.dev = &pdev->dev;
	jz4770->chip.ops = &jz4770_pwm_ops;
	jz4770->chip.npwm = NUM_PWM;
	jz4770->chip.base = -1;

	ret = pwmchip_add(&jz4770->chip);
	if (ret < 0) {
		clk_put(jz4770->clk);
		return ret;
	}

	platform_set_drvdata(pdev, jz4770);

	return 0;
}

static int __devexit jz4770_pwm_remove(struct platform_device *pdev)
{
	struct jz4770_pwm_chip *jz4770 = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&jz4770->chip);
	if (ret < 0)
		return ret;

	clk_put(jz4770->clk);

	return 0;
}

static struct platform_driver jz4770_pwm_driver = {
	.driver = {
		.name = "jz4770-pwm",
		.owner = THIS_MODULE,
	},
	.probe = jz4770_pwm_probe,
	.remove = __devexit_p(jz4770_pwm_remove),
};
module_platform_driver(jz4770_pwm_driver);

MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_DESCRIPTION("Ingenic JZ4770 PWM driver");
MODULE_ALIAS("platform:jz4770-pwm");
MODULE_LICENSE("GPL");

/*
 * gcw0.c  --  SoC audio for GCW Zero
 *
 * PISCES JZ4770 board support: (was used as a base)
 * Copyright (C) Ingenic Semiconductor Inc.
 *
 * GCW Zero specific board support:
 * Copyright (C) 2012, Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-jz4770/jz4770gpio.h>


#define GCW0_AVOUT_DETECT_GPIO	GPF(21)
#define GCW0_AVOUT_GPIO		GPF(3)
#define GCW0_SPEAKER_GPIO	GPF(20)


/* AV-out jack: plug insert detection */

static struct snd_soc_jack gcw0_avout_jack;

static struct snd_soc_jack_pin gcw0_avout_jack_pins[] = {
	{
		.pin		= "AV-out",
		.mask		= SND_JACK_HEADPHONE,
	},
	{
		.pin		= "Speakers",
		.mask		= SND_JACK_HEADPHONE,
		.invert		= 1,
	},
};

static struct snd_soc_jack_gpio gcw0_avout_jack_gpios[] = {
	{
		.name		= "AV-out Detect",
		.report		= SND_JACK_HEADPHONE,
		.gpio		= GCW0_AVOUT_DETECT_GPIO,
		.invert		= 1,
		.debounce_time	= 200,
	},
};

/* AV-out and speaker switches */

static int gcw0_avout_event(struct snd_soc_dapm_widget *widget,
			    struct snd_kcontrol *ctrl, int event)
{
	gpio_set_value(GCW0_AVOUT_GPIO, !SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static int gcw0_speaker_event(struct snd_soc_dapm_widget *widget,
			      struct snd_kcontrol *ctrl, int event)
{
	gpio_set_value(GCW0_SPEAKER_GPIO, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static const struct snd_kcontrol_new gcw0_controls[] = {
	SOC_DAPM_PIN_SWITCH("AV-out"),
	SOC_DAPM_PIN_SWITCH("Speakers"),
};

/* GCW0 machine dapm widgets */
static const struct snd_soc_dapm_widget gcw0_widgets[] = {
	SND_SOC_DAPM_HP("AV-out", gcw0_avout_event),
	SND_SOC_DAPM_SPK("Speakers", gcw0_speaker_event),
	SND_SOC_DAPM_MIC("Mic1 Jack", NULL),
	SND_SOC_DAPM_MIC("Mic2 Jack", NULL),
};

/* GCW0 machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route gcw0_routes[] = {
	/* Destination Widget(sink)  <=== Path Name <=== Source Widget */

	{ "AV-out", NULL, "LHPOUT" },
	{ "AV-out", NULL, "RHPOUT" },

	{ "Speakers", NULL, "LOUT" },
	{ "Speakers", NULL, "ROUT" },

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{ "MIC1P", NULL, "Mic Bias" },
	{ "MIC1N", NULL, "Mic Bias" }, /* no such connection, but not harm */
	{ "Mic Bias", NULL, "Mic1 Jack" },

	{ "MIC2P", NULL, "Mic Bias" },
	{ "MIC2N", NULL, "Mic Bias" },
	{ "Mic Bias", NULL, "Mic2 Jack" },
};

/*
 * GCW0 for a jz_icdc as connected on jz4770 Device
 */
static int gcw0_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	/* set up codec pins not used */
	/* on Pisces board, linein pin are not connected */
	snd_soc_dapm_nc_pin(dapm, "LLINEIN");
	snd_soc_dapm_nc_pin(dapm, "RLINEIN");

	/* set up AV-out plug detection */
	snd_soc_jack_new(codec, "AV-out Jack",
			 SND_JACK_VIDEOOUT | SND_JACK_HEADPHONE,
			 &gcw0_avout_jack);
	snd_soc_jack_add_pins(&gcw0_avout_jack,
			      ARRAY_SIZE(gcw0_avout_jack_pins),
			      gcw0_avout_jack_pins);
	snd_soc_jack_add_gpios(&gcw0_avout_jack,
			       ARRAY_SIZE(gcw0_avout_jack_gpios),
			       gcw0_avout_jack_gpios);

	/* set endpoints to default mode */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
					   SND_SOC_DAIFMT_NB_NF |
					   SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cpu dai format: %d\n", ret);
		return ret;
	}

	return 0;
}

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link gcw0_dai = {
	.name			= "JZ_ICDC",
	.stream_name		= "JZ_ICDC",
	.cpu_dai_name		= "jz4770-i2s",
	.platform_name		= "jz4770-pcm-audio",
	.codec_dai_name		= "jz4770-hifi",
	.codec_name		= "jz4770-icdc",
	.init			= gcw0_codec_init,
};

static struct snd_soc_card gcw0_card = {
	.name			= "GCW0",
	.owner			= THIS_MODULE,
	.dai_link		= &gcw0_dai,
	.num_links		= 1,

	.controls		= gcw0_controls,
	.num_controls		= ARRAY_SIZE(gcw0_controls),

	.dapm_widgets		= gcw0_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(gcw0_widgets),
	.dapm_routes		= gcw0_routes,
	.num_dapm_routes	= ARRAY_SIZE(gcw0_routes),
};

static int __devinit gcw0_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &gcw0_card;
	int ret;

	ret = devm_gpio_request(&pdev->dev, GCW0_AVOUT_GPIO, "AV-out");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request AV-out GPIO(%d): %d\n",
			GCW0_AVOUT_GPIO, ret);
		return ret;
	}

	ret = devm_gpio_request(&pdev->dev, GCW0_SPEAKER_GPIO, "Speaker");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request speaker GPIO(%d): %d\n",
			GCW0_SPEAKER_GPIO, ret);
		return ret;
	}

	__gpio_disable_pull(GCW0_AVOUT_DETECT_GPIO);
	gpio_direction_output(GCW0_AVOUT_GPIO, 1);
	gpio_direction_output(GCW0_SPEAKER_GPIO, 0);

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int __devexit gcw0_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_jack_free_gpios(&gcw0_avout_jack,
				ARRAY_SIZE(gcw0_avout_jack_gpios),
				gcw0_avout_jack_gpios);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver gcw0_driver = {
	.driver		= {
		.name	= "gcw0-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= gcw0_probe,
	.remove		= __devexit_p(gcw0_remove),
};

module_platform_driver(gcw0_driver);

MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("ALSA SoC GCW0 Audio support");
MODULE_LICENSE("GPL v2");

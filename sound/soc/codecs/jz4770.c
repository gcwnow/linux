/*
 * Copyright (C) Ingenic Semiconductor Inc.
 * Original driver by Lutts Wolf <slcao@ingenic.cn>.
 *
 * Copyright (C) 2012, Maarten ter Huurne <maarten@treewalker.org>
 * Updated to match ALSA changes and restructured to better fit driver model.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <asm/div64.h>
#include <asm/types.h>
#include <asm/mach-jz4770/jz4770aic.h>
#include <asm/mach-jz4770/jz4770cpm.h>

#include "jz4770.h"


/*
 * Note: On the GCW Zero, left and right are the wrong way around. This define
 *       toggles the code that compensates for it. However, I don't know where
 *       exactly in the audio flow the problem is, so it is possible more
 *       compensation is needed.
 */
#define SWAP_LR 1


static int jz_icdc_debug = 0;
module_param(jz_icdc_debug, int, 0644);

#define DEBUG_MSG(msg...)			\
	do {					\
		if (jz_icdc_debug)		\
			printk("ICDC: " msg);	\
	} while(0)

/* codec private data */
struct jz_icdc {
	struct regmap *regmap;
	void __iomem *base;
};

static int hpout_enable = 0;
//static int lineout_enable = 0;
static int bypass_to_hp = 0;
static int bypass_to_lineout = 0;

__attribute__((__unused__)) static void dump_aic_regs(const char *func, int line)
{
	char *regname[] = {"aicfr","aiccr","aiccr1","aiccr2","i2scr","aicsr","acsr","i2ssr",
				"accar", "accdr", "acsar", "acsdr", "i2sdiv"};
	int i;
	unsigned int addr;

	printk("AIC regs dump, %s:%d:\n", func, line);
	for (i = 0; i <= 0x30; i += 4) {
		addr = AIC_BASE + i;
		printk("%s\t0x%08x -> 0x%08x\n", regname[i/4], addr, *(unsigned int *)addr);
	}
}

static unsigned int jz_icdc_read_reg(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	struct jz_icdc *jz_icdc = snd_soc_codec_get_drvdata(codec);
	unsigned int val;

	regmap_read(jz_icdc->regmap, reg, &val);

	return val;
}

static void jz_icdc_update_reg(struct snd_soc_codec *codec, unsigned int reg,
			       unsigned int lsb, u8 mask, u8 val)
{
	struct jz_icdc *jz_icdc = snd_soc_codec_get_drvdata(codec);

	regmap_update_bits(jz_icdc->regmap, reg, mask << lsb, val << lsb);
}

static void turn_on_sb_hp(struct snd_soc_codec *codec)
{
	if (jz_icdc_read_reg(codec, JZ_ICDC_CR_HP) & (1 << 4)) {
		/* set cap-less */
		jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 3, 0x1, 0);

		/* power on sb hp */
		jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 4, 0x1, 0);


		while( !(jz_icdc_read_reg(codec, JZ_ICDC_IFR) & (1 << 3)))
			mdelay(10);

		mdelay(1);

		/* clear RUP flag */
		jz_icdc_update_reg(codec, JZ_ICDC_IFR, 3, 0x1, 1);
	}
}

static void turn_off_sb_hp(struct snd_soc_codec *codec)
{
	if (!(jz_icdc_read_reg(codec, JZ_ICDC_CR_HP) & (1 << 4))) {

		/* set cap-couple */
		jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 3, 0x1, 1);
		/* power on sb hp */
		jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 4, 0x1, 1);


		while( !(jz_icdc_read_reg(codec, JZ_ICDC_IFR) & (1 << 2)))
			mdelay(10);

		mdelay(1);

		/* clear RUP flag */
		jz_icdc_update_reg(codec, JZ_ICDC_IFR, 2, 0x1, 1);
	}
}

static void turn_on_dac(struct snd_soc_codec *codec)
{
	/* DAC_MUTE */
	if (jz_icdc_read_reg(codec, JZ_ICDC_CR_DAC) & (1 << 7)) {
		/* power on sb hp */
		jz_icdc_update_reg(codec, JZ_ICDC_CR_DAC, 7, 0x1, 0);


		while( !(jz_icdc_read_reg(codec, JZ_ICDC_IFR) & (1 << 1)))
			mdelay(10);

		mdelay(1);

		/* clear RUP flag */
		jz_icdc_update_reg(codec, JZ_ICDC_IFR, 1, 0x1, 1);
	}
}

static void turn_off_dac(struct snd_soc_codec *codec)
{
	if (!(jz_icdc_read_reg(codec, JZ_ICDC_CR_DAC) & (1 << 7))) {
		/* power on sb hp */
		jz_icdc_update_reg(codec, JZ_ICDC_CR_DAC, 7, 0x1, 1);


		while( !(jz_icdc_read_reg(codec, JZ_ICDC_IFR) & (1 << 0)))
			mdelay(10);

		mdelay(1);

		/* clear RUP flag */
		jz_icdc_update_reg(codec, JZ_ICDC_IFR, 0, 0x1, 1);
	}
}

static int jz_icdc_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		jz_icdc_update_reg(codec, JZ_ICDC_CR_MIC, 0, 0x1, 0);
		mdelay(10);
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		jz_icdc_update_reg(codec, JZ_ICDC_CR_MIC, 0, 0x1, 1);
		mdelay(10);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static int jz_icdc_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	bool playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	int bit_width;
	int duplicate; /* stereo-to-mono for DAC path */
	int speed;

	/* check bit width */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_width = 0;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		bit_width = 1;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bit_width = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		bit_width = 3;
		break;
	default:
		return -EINVAL;
	}

	/* check channels */
	switch (params_channels(params)) {
	case 1:
		duplicate = 1;
		break;
	case 2:
		duplicate = 0;
		break;
	default:
		return -EINVAL;
	}

	/* check sample rate */
	switch (params_rate(params)) {
	case 96000:
		speed = 0;
		break;
	case 48000:
		speed = 1;
		break;
	case 44100:
		speed = 2;
		break;
	case 32000:
		speed = 3;
		break;
	case 24000:
		speed = 4;
		break;
	case 22050:
		speed = 5;
		break;
	case 16000:
		speed = 6;
		break;
	case 12000:
		speed = 7;
		break;
	case 11025:
		speed = 8;
		break;
	case 8000:
		speed = 9;
		break;
	default:
		return -EINVAL;
	}

	/* apply bit width */
	if (playback)
		jz_icdc_update_reg(codec, JZ_ICDC_AICR_DAC, 6, 0x3, bit_width);
	else
		jz_icdc_update_reg(codec, JZ_ICDC_AICR_ADC, 6, 0x3, bit_width);

	/* apply channels */
	if (playback)
		jz_icdc_update_reg(codec, JZ_ICDC_CR_DAC, 6, 0x1, duplicate);

	/* apply sample rate */
	if (playback)
		jz_icdc_update_reg(codec, JZ_ICDC_FCR_DAC, 0, 0xf, speed);
	else
		jz_icdc_update_reg(codec, JZ_ICDC_FCR_ADC, 0, 0xf, speed);

	return 0;
}

static void jz_icdc_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	DEBUG_MSG("enter jz_icdc_shutdown, playback = %d\n", playback);

	if (playback || bypass_to_hp || bypass_to_lineout) {
		turn_off_dac(codec);

		/* anti-pop workaround */
		__aic_write_tfifo(0x0);
		__aic_write_tfifo(0x0);
		__i2s_enable_replay();
		__i2s_enable();
		mdelay(1);
		__i2s_disable_replay();
		__i2s_disable();

		if (hpout_enable)
			turn_off_sb_hp(codec);

#if 0
		/* mute headphone */
		jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 7, 0x1, 1);
		mdelay(1);
#endif

		/* power down SB_DAC */
		jz_icdc_update_reg(codec, JZ_ICDC_CR_DAC, 4, 0x1, 1);
		mdelay(10);
	}

	mdelay(10);
}

static int jz_icdc_pcm_trigger(struct snd_pcm_substream *substream,
			      int cmd, struct snd_soc_dai *dai) {
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	DEBUG_MSG("enter %s:%d substream = %s bypass_to_hp = %d bypassto_lineout = %d cmd = %d\n",
		  __func__, __LINE__,
		  (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture"),
		  bypass_to_hp, bypass_to_lineout,
		  cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ||
		    bypass_to_hp || bypass_to_lineout){
			/* SB_DAC */
			jz_icdc_update_reg(codec, JZ_ICDC_CR_DAC, 4, 0x1, 0);
			mdelay(1);

#if 0
			/* HP_MUTE */
			jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 7, 0x1, 0);
			mdelay(1);
#endif

			if (hpout_enable)
				turn_on_sb_hp(codec);

			turn_on_dac(codec);

			//dump_aic_regs(__func__, __LINE__);
		} else {
			jz_icdc_set_bias_level(codec, SND_SOC_BIAS_ON);
			mdelay(2);

			//dump_aic_regs(__func__, __LINE__);
		}

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* do nothing */
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int jz_icdc_mute(struct snd_soc_dai *dai, int mute)
{
	jz_icdc_update_reg(dai->codec, JZ_ICDC_CR_DAC, 7, 0x1, mute);
	return 0;
}

static int jz_icdc_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	return 0;
}

#define JZ_ICDC_RATES (SNDRV_PCM_RATE_8000  | SNDRV_PCM_RATE_11025 | \
		       SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		       SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		       SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

#define JZ_ICDC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE |SNDRV_PCM_FMTBIT_S24_3LE)

static const struct snd_soc_dai_ops jz_icdc_dai_ops = {
	.hw_params	= jz_icdc_hw_params,
	.trigger	= jz_icdc_pcm_trigger,
	.shutdown       = jz_icdc_shutdown,
	.digital_mute	= jz_icdc_mute,
	.set_sysclk	= jz_icdc_set_dai_sysclk,
};

static struct snd_soc_dai_driver jz_icdc_dai = {
	.name = "jz4770-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZ_ICDC_RATES,
		.formats = JZ_ICDC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = JZ_ICDC_RATES,
		.formats = JZ_ICDC_FORMATS,
	},
	.ops = &jz_icdc_dai_ops,
};

/* unit: 0.01dB */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -3100, 100, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(out_tlv, -2500, 100, 0);
static const DECLARE_TLV_DB_SCALE(mic1_boost_tlv, 0, 400, 0);
static const DECLARE_TLV_DB_SCALE(mic2_boost_tlv, 0, 400, 0);
static const DECLARE_TLV_DB_SCALE(linein_tlv, -2500, 100, 0);

static const struct snd_kcontrol_new jz_icdc_snd_controls[] = {
	/* playback gain control */
	SOC_DOUBLE_R_TLV("PCM Volume",
#if SWAP_LR
			 JZ_ICDC_GCR_DACR, JZ_ICDC_GCR_DACL,
#else
			 JZ_ICDC_GCR_DACL, JZ_ICDC_GCR_DACR,
#endif
			 0, 31, 1, dac_tlv),
	SOC_DOUBLE_R_TLV("Master Playback Volume",
#if SWAP_LR
			 JZ_ICDC_GCR_HPR, JZ_ICDC_GCR_HPL,
#else
			 JZ_ICDC_GCR_HPL, JZ_ICDC_GCR_HPR,
#endif
			 0, 31, 1, out_tlv),

	/* record gain control */
	SOC_DOUBLE_R_TLV("ADC Capture Volume",
			 JZ_ICDC_GCR_ADCL, JZ_ICDC_GCR_ADCR, 0, 23, 0, adc_tlv),

	SOC_SINGLE_TLV("Mic1 Capture Volume",
		       JZ_ICDC_GCR_MIC1, 0, 5, 0, mic1_boost_tlv),
	SOC_SINGLE_TLV("Mic2 Capture Volume",
		       JZ_ICDC_GCR_MIC2, 0, 5, 0, mic2_boost_tlv),

	SOC_DOUBLE_R_TLV("Linein Bypass Capture Volume",
			 JZ_ICDC_GCR_LIBYL, JZ_ICDC_GCR_LIBYR, 0, 31, 1, linein_tlv),
};

static int hpout_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event) {
	DEBUG_MSG("enter %s, event = 0x%08x\n", __func__, event);

	mdelay(1);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		hpout_enable = 1;
		break;

	case SND_SOC_DAPM_POST_PMD:
		hpout_enable = 0;
		break;
	}

	return 0;
}

static int lineout_event(struct snd_soc_dapm_widget *w,
		     struct snd_kcontrol *kcontrol, int event) {
	struct snd_soc_codec *codec = w->codec;

	DEBUG_MSG("enter %s, event = 0x%08x\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		jz_icdc_update_reg(codec, JZ_ICDC_CR_LO, 7, 0x1, 0);
		msleep(1);
		//lineout_enable = 1;
		break;

	case SND_SOC_DAPM_POST_PMD:
		jz_icdc_update_reg(codec, JZ_ICDC_CR_LO, 7, 0x1, 1);
		msleep(1);
		//lineout_enable = 0;
		break;
	}

	return 0;
}

static const char *jz_icdc_mic_sel[] = {
	"Mono", "Stereo"
};
static const char *jz_icdc_input_sel[] = {
	"Mic 1", "Mic 2", "Line In"
};
static const char *jz_icdc_output_sel[] = {
	"Mic 1", "Mic 2", "Line In", "Stereo DAC"
};

static int micbias_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event) {

	//struct snd_soc_codec *codec = w->codec;
	//u8 *cache = codec->reg_cache;

	switch (event) {
	case SND_SOC_DAPM_POST_REG:
		msleep(10);
		break;
	default:
		break;
	}

	return 0;
}

static int adc_poweron_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event) {

	//struct snd_soc_codec *codec = w->codec;
	//u8 *cache = codec->reg_cache;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		msleep(1000);
		break;
	default:
		break;
	}

	return 0;
}

static const SOC_ENUM_SINGLE_DECL(jz_icdc_mic_enum,
				  JZ_ICDC_CR_MIC, 7, jz_icdc_mic_sel);
static const SOC_ENUM_SINGLE_DECL(jz_icdc_hp_enum,
				  JZ_ICDC_CR_HP,  0, jz_icdc_output_sel);
static const SOC_ENUM_SINGLE_DECL(jz_icdc_lo_enum,
				  JZ_ICDC_CR_LO,  0, jz_icdc_output_sel);
static const SOC_ENUM_SINGLE_DECL(jz_icdc_adc_enum,
				  JZ_ICDC_CR_ADC, 0, jz_icdc_input_sel);

static const struct snd_kcontrol_new icdc_mic_mux_controls =
	SOC_DAPM_ENUM("Route", jz_icdc_mic_enum);

static const struct snd_kcontrol_new icdc_hp_mux_controls =
	SOC_DAPM_ENUM("Route", jz_icdc_hp_enum);

static const struct snd_kcontrol_new icdc_lo_mux_controls =
	SOC_DAPM_ENUM("Route", jz_icdc_lo_enum);

static const struct snd_kcontrol_new icdc_adc_controls =
	SOC_DAPM_ENUM("Route", jz_icdc_adc_enum);


static const struct snd_soc_dapm_widget jz_icdc_dapm_widgets[] = {
	SND_SOC_DAPM_PGA_E("HP Out", JZ_ICDC_CR_HP, 7, 1, NULL, 0,
			   hpout_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("Line Out", JZ_ICDC_CR_LO, 4, 1, NULL, 0,
			   lineout_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA("Line Input", JZ_ICDC_CR_LI, 0, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Mic1 Input", JZ_ICDC_CR_MIC, 4, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Mic2 Input", JZ_ICDC_CR_MIC, 5, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Linein Bypass", JZ_ICDC_CR_LI, 4, 1, NULL, 0),

	SND_SOC_DAPM_ADC_E("ADC", "HiFi Capture", JZ_ICDC_CR_ADC, 4, 1,
			   adc_poweron_event,
			   SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_DAC("DAC", "HiFi Playback", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_SUPPLY("Mic Bias", JZ_ICDC_CR_MIC, 0, 1,
			       micbias_event,
			       SND_SOC_DAPM_POST_REG),

	SND_SOC_DAPM_MUX("Microphone Mux", SND_SOC_NOPM, 0, 0,
			 &icdc_mic_mux_controls),

	SND_SOC_DAPM_MUX("Playback HP Mux", SND_SOC_NOPM, 0, 0,
			 &icdc_hp_mux_controls),

	SND_SOC_DAPM_MUX("Playback Lineout Mux", SND_SOC_NOPM, 0, 0,
			 &icdc_lo_mux_controls),

	SND_SOC_DAPM_MUX("Capture Mux", SND_SOC_NOPM, 0, 0,
			 &icdc_adc_controls),

	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),

	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),

	/* FIXME: determine MICDIFF here. */
	SND_SOC_DAPM_INPUT("MIC1P"),
	SND_SOC_DAPM_INPUT("MIC1N"),
	SND_SOC_DAPM_INPUT("MIC2P"),
	SND_SOC_DAPM_INPUT("MIC2N"),

	SND_SOC_DAPM_INPUT("LLINEIN"),
	SND_SOC_DAPM_INPUT("RLINEIN"),
};

static const struct snd_soc_dapm_route jz_icdc_dapm_routes[] = {
	/* Destination Widget  <=== Path Name <=== Source Widget */
	{ "Mic1 Input", NULL, "MIC1P" },
	{ "Mic1 Input", NULL, "MIC1N" },

	{ "Mic2 Input", NULL, "MIC2P" },
	{ "Mic2 Input", NULL, "MIC2N" },

	{ "Line Input", NULL, "LLINEIN" },
	{ "Line Input", NULL, "RLINEIN" },


	{ "Microphone Mux", "Stereo", "Mic1 Input" },
	{ "Microphone Mux", "Stereo", "Mic2 Input" },


	{ "Capture Mux", "Mic 1", "Mic1 Input" },
	{ "Capture Mux", "Mic 1", "Microphone Mux" },
	{ "Capture Mux", "Mic 2", "Mic2 Input" },
	{ "Capture Mux", "Mic 2", "Microphone Mux" },
	{ "Capture Mux", "Line In", "Line Input" },

	{ "ADC", NULL, "Capture Mux" },


	{ "Playback HP Mux", "Mic 1", "Mic1 Input" },
	{ "Playback HP Mux", "Mic 1", "Microphone Mux" },
	{ "Playback HP Mux", "Mic 2", "Mic2 Input" },
	{ "Playback HP Mux", "Mic 2", "Microphone Mux" },
	{ "Playback HP Mux", "Line In", "Line Input" },
	{ "Playback HP Mux", "Stereo DAC", "DAC" },

	{ "HP Out", NULL, "Playback HP Mux" },

	{ "LHPOUT", NULL, "HP Out"},
	{ "RHPOUT", NULL, "HP Out"},


	{ "Playback Lineout Mux", "Mic 1", "Mic1 Input" },
	{ "Playback Lineout Mux", "Mic 1", "Microphone Mux" },
	{ "Playback Lineout Mux", "Mic 2", "Mic2 Input" },
	{ "Playback Lineout Mux", "Mic 2", "Microphone Mux" },
	{ "Playback Lineout Mux", "Line In", "Line Input" },
	{ "Playback Lineout Mux", "Stereo DAC", "DAC" },

	{ "Line Out", NULL, "Playback Lineout Mux" },

	{ "LOUT", NULL, "Line Out"},
	{ "ROUT", NULL, "Line Out"},
};

#ifdef CONFIG_PM_SLEEP

static int jz_icdc_suspend(struct snd_soc_codec *codec)
{
	return jz_icdc_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static int jz_icdc_resume(struct snd_soc_codec *codec)
{
	return jz_icdc_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
}

#else
#define jz_icdc_suspend NULL
#define jz_icdc_resume NULL
#endif

static int jz_icdc_dev_probe(struct snd_soc_codec *codec)
{
	struct jz_icdc *jz_icdc = snd_soc_codec_get_drvdata(codec);
	struct regmap *regmap = jz_icdc->regmap;

	cpm_start_clock(CGM_AIC);
	mdelay(1);

	/* Collect updates for later sending. */
	regcache_cache_only(regmap, true);

	//dump_aic_regs(__func__, __LINE__);

	/* default: DAC */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 0, 0x3, 0x3);

	/* default: DAC */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_LO, 0, 0x3, 0x3);

	/* default: L(mic1) + R(mic1) */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_ADC, 0, 0x3, 0x0);
	/* mic mono */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_MIC, 7, 0x1, 0);

	/* init codec params */
	/* ADC/DAC: serial + i2s */
	jz_icdc_update_reg(codec, JZ_ICDC_AICR_ADC, 0, 0x3, 0x3);
	jz_icdc_update_reg(codec, JZ_ICDC_AICR_DAC, 0, 0x3, 0x3);

	/* The generated IRQ is a high level */
	jz_icdc_update_reg(codec, JZ_ICDC_ICR, 6, 0x3, 0x0);
	jz_icdc_update_reg(codec, JZ_ICDC_IMR, 0, 0xff,
			   ((1 << 0) | (1 << 1) |
			    (1 << 2) | (1 << 3) | (1 << 5)));

	/* 12M */
	jz_icdc_update_reg(codec, JZ_ICDC_CCR, 0, 0xf, 0x0);

	/* 0: 16ohm/220uF, 1: 10kohm/1uF */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 6, 0x1, 0);

	/* disable AGC */
	jz_icdc_update_reg(codec, JZ_ICDC_AGC1, 7, 0x1, 0);

	/* default to MICDIFF */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_MIC, 6, 0x1, 1);

	/* mute lineout/HP */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_LO, 7, 0x1, 1);
	jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 7, 0x1, 1);

	/* DAC lrswap */
#if SWAP_LR
	jz_icdc_update_reg(codec, JZ_ICDC_CR_DAC, 3, 0x1, 0);
#else
	jz_icdc_update_reg(codec, JZ_ICDC_CR_DAC, 3, 0x1, 1);
#endif

	/* ADC lrswap */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_ADC, 3, 0x1, 1);

	/* default to cap-less mode(0) */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_HP, 3, 0x1, 0);

	/* Send collected updates. */
	regcache_cache_only(regmap, false);
	regcache_sync(regmap);

	/* Reset all interrupt flags. */
	regmap_write(regmap, JZ_ICDC_IFR, 0xFF);

	/* These steps are too time consuming, so we do it here */
	/* clr SB */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_VIC,  0, 0x1, 0);
	mdelay(300);

	/* clr SB_SLEEP */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_VIC, 1, 0x1, 0);
	mdelay(400);

	return jz_icdc_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
}

static int jz_icdc_dev_remove(struct snd_soc_codec *codec)
{
	/* clr SB_SLEEP */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_VIC, 1, 0x1, 1);
	mdelay(10);

	/* clr SB */
	jz_icdc_update_reg(codec, JZ_ICDC_CR_VIC, 0, 0x1, 1);

	return jz_icdc_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

const struct snd_soc_codec_driver jz_icdc_soc_codec_dev = {
	.probe			= jz_icdc_dev_probe,
	.remove			= jz_icdc_dev_remove,
	.suspend		= jz_icdc_suspend,
	.resume			= jz_icdc_resume,

	.controls		= jz_icdc_snd_controls,
	.num_controls		= ARRAY_SIZE(jz_icdc_snd_controls),
	.dapm_widgets		= jz_icdc_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(jz_icdc_dapm_widgets),
	.dapm_routes		= jz_icdc_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(jz_icdc_dapm_routes),

	.set_bias_level		= jz_icdc_set_bias_level,
};

static bool jz_icdc_volatile(struct device *dev, unsigned int reg)
{
	return reg == JZ_ICDC_SR || reg == JZ_ICDC_IFR;
}

static bool jz_icdc_readable(struct device *dev, unsigned int reg)
{
	return reg != JZ_ICDC_MISSING_REG1 && reg != JZ_ICDC_MISSING_REG2;
}

static bool jz_icdc_writeable(struct device *dev, unsigned int reg)
{
	return reg != JZ_ICDC_SR &&
	       reg != JZ_ICDC_MISSING_REG1 && reg != JZ_ICDC_MISSING_REG2;
}

static int jz_icdc_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	volatile unsigned int dummy;

	while (__icdc_rgwr_ready());

	__icdc_set_addr(reg);

	/* wait 4+ cycle */
	dummy = __icdc_get_value();
	dummy = __icdc_get_value();
	dummy = __icdc_get_value();
	dummy = __icdc_get_value();
	dummy = __icdc_get_value();

	*val = __icdc_get_value();

	return 0;
}

static int jz_icdc_reg_write(void *context, unsigned int reg, unsigned int val)
{
	while (__icdc_rgwr_ready());

	REG_ICDC_RGADW = ICDC_RGADW_RGWR | (reg << ICDC_RGADW_RGADDR_LSB) | val;

	while (__icdc_rgwr_ready());

	return 0;
}

static const u8 jz_icdc_reg_defaults[] = {
	0x00, 0xC3, 0xC3, 0x90, 0x98, 0xFF, 0x90, 0xB1,
	0x11, 0x10, 0x00, 0x03, 0x00, 0x00, 0x40, 0x00,
	0xFF, 0x00, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x34,
	0x07, 0x44, 0x1F, 0x00
};

static struct regmap_config jz_icdc_regmap_config = {
	.reg_bits = 7,
	.val_bits = 8,

	.max_register = JZ_ICDC_MAX_NUM - 1,
	.volatile_reg = jz_icdc_volatile,
	.readable_reg = jz_icdc_readable,
	.writeable_reg = jz_icdc_writeable,

	.reg_read = jz_icdc_reg_read,
	.reg_write = jz_icdc_reg_write,

	.reg_defaults_raw = jz_icdc_reg_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(jz_icdc_reg_defaults),
	.use_single_rw = 1,
	.cache_type = REGCACHE_FLAT,
};

static int jz_icdc_probe(struct platform_device *pdev)
{
	int ret;
	struct jz_icdc *jz_icdc;
	struct resource *mem;

	jz_icdc = devm_kzalloc(&pdev->dev, sizeof(*jz_icdc), GFP_KERNEL);
	if (!jz_icdc)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "Failed to get mmio memory resource\n");
		return -ENOENT;
	}

	jz_icdc->base = devm_request_and_ioremap(&pdev->dev, mem);
	if (!jz_icdc->base) {
		dev_err(&pdev->dev, "Failed to request and map mmio memory\n");
		return -EBUSY;
	}

	jz_icdc->regmap = devm_regmap_init(&pdev->dev, NULL, NULL,
					   &jz_icdc_regmap_config);
	if (IS_ERR(jz_icdc->regmap))
		return PTR_ERR(jz_icdc->regmap);

	platform_set_drvdata(pdev, jz_icdc);

	ret = snd_soc_register_codec(&pdev->dev, &jz_icdc_soc_codec_dev,
				     &jz_icdc_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);
		return ret;
	}

	return 0;
}

static int jz_icdc_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver jz4770_icdc_driver = {
	.probe			= jz_icdc_probe,
	.remove			= jz_icdc_remove,
	.driver			= {
		.name		= "jz4770-icdc",
		.owner		= THIS_MODULE,
	},
};

module_platform_driver(jz4770_icdc_driver);

MODULE_DESCRIPTION("Jz4770 Internal Codec Driver");
MODULE_AUTHOR("Lutts Wolf<slcao@ingenic.cn>");
MODULE_LICENSE("GPL");

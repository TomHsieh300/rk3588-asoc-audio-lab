// SPDX-License-Identifier: GPL-2.0-only
/*
 * tom-es8388.c  --  ES8388 ALSA SoC Audio driver (practice, DAC-only)
 *
 * Author: Tom Hsieh <hungen3108@gmail.com>
 *
 * === Always-On Anti-Pop Architecture ===
 *
 * This driver differs from upstream es8328.c in one key way: all analog
 * power blocks (R02, R04) stay on between play and stop. Only the DAC
 * mute bit (R19[2]) toggles. This eliminates pop noise caused by analog
 * power state changes during play/stop transitions.
 *
 * Trade-off: ~4-5mA idle current vs pop-free operation.
 *
 * Register ownership:
 *
 *   set_bias_level:  R00 (VMID), R01 (protection), R02 (digital power),
 *                    R04 (DAC + output amp)
 *   reg_defaults:    R17/R18 (DAC format/rate), R19 (mute/ramp),
 *                    R1C (clickfree), R2D (VROI),
 *                    R27/R2A (mixer initial routing), R2E/R2F (output PGA)
 *   mute_stream:     R19[2] (DAC mute bit only)
 *   kcontrols:       R1A/R1B (DAC digital volume), R2E/R2F (output PGA)
 *   DAPM switches:   R27[7]/R2A[7] (mixer playback switch, tinymix control)
 *   hw_params:       R17 (word length), R18 (MCLK ratio)
 *
 * Power state transitions:
 *
 *   Boot:    OFF -> STANDBY: 5k charge(100ms) -> R04 on(50ms) -> 50k
 *   Play:    STANDBY -> PREPARE -> ON: R02=0x00, VMID stays 50k
 *   Stop:    ON -> PREPARE -> STANDBY: mute R19[2], VMID stays 50k
 *   Suspend: STANDBY -> OFF: R02=0xFF -> R04=0xC0 -> VMID off
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

/* ========== Register definitions ========== */

#define ES8388_SUPPLY_NUM		4
#define ES8388_REG_MAX			0x35

/* Timing constants (ms) */
#define ES8388_VMID_CHARGE_MS		100	/* 5k fast charge duration */
#define ES8388_DACPOWER_SETTLE_MS	50	/* DAC + output amp settle */

/* R00: Chip Control 1 */
#define ES8388_CONTROL1			0x00
#define ES8388_CONTROL1_VMIDSEL_MASK	(3 << 0)
#define ES8388_CONTROL1_VMIDSEL_OFF	(0 << 0)
#define ES8388_CONTROL1_VMIDSEL_50k	(1 << 0)
#define ES8388_CONTROL1_VMIDSEL_500k	(2 << 0)
#define ES8388_CONTROL1_VMIDSEL_5k	(3 << 0)
#define ES8388_CONTROL1_ENREF		(1 << 2)

/* R01: Chip Control 2 */
#define ES8388_CONTROL2			0x01
#define ES8388_CONTROL2_OVERCURRENT	(1 << 6)
#define ES8388_CONTROL2_THERMAL		(1 << 7)

/* R02: Chip Power Management */
#define ES8388_CHIPPOWER		0x02
#define ES8388_CHIPPOWER_DACVREF_OFF	0
#define ES8388_CHIPPOWER_ADCVREF_OFF	1
#define ES8388_CHIPPOWER_DACDLL_OFF	2
#define ES8388_CHIPPOWER_ADCDLL_OFF	3
#define ES8388_CHIPPOWER_DACSTM_RESET	4
#define ES8388_CHIPPOWER_ADCSTM_RESET	5
#define ES8388_CHIPPOWER_DACDIG_OFF	6
#define ES8388_CHIPPOWER_ADCDIG_OFF	7
#define ES8388_CHIPPOWER_ALL_ON		0x00
#define ES8388_CHIPPOWER_ALL_OFF	0xFF

/* R04: DAC Power Management */
#define ES8388_DACPOWER			0x04
#define ES8388_DACPOWER_LDAC_OFF	7
#define ES8388_DACPOWER_RDAC_OFF	6
#define ES8388_DACPOWER_LOUT1_ON	5
#define ES8388_DACPOWER_ROUT1_ON	4
/* Composite values for set_bias_level */
#define ES8388_DACPOWER_LOUT1_ROUT1	0x30	/* DAC on + LOUT1/ROUT1 on */
#define ES8388_DACPOWER_ALL_OFF		0xC0	/* DAC off, all outputs off */

/* R08: Master Mode */
#define ES8388_MASTERMODE		0x08
#define ES8388_MASTERMODE_MSC		(1 << 7)
#define ES8388_MASTERMODE_MCLKDIV2	(1 << 6)

/* R17: DAC Control 1 */
#define ES8388_DACCONTROL1		0x17
#define ES8388_DACCONTROL1_DACWL_SHIFT	3
#define ES8388_DACCONTROL1_DACWL_MASK	(7 << 3)
#define ES8388_DACCONTROL1_DACFMT_MASK	(3 << 1)

/* R18: DAC Control 2 */
#define ES8388_DACCONTROL2		0x18
#define ES8388_DACCONTROL2_RATEMASK	(0x1f << 0)

/* R19: DAC Control 3 */
#define ES8388_DACCONTROL3		0x19
#define ES8388_DACCONTROL3_DACMUTE	(1 << 2)
#define ES8388_DACCONTROL3_DACSOFTRAMP	(1 << 5)
#define ES8388_DACCONTROL3_RAMPRATE_MASK	(3 << 6)
#define ES8388_DACCONTROL3_RAMPRATE_4LRCK	(0 << 6)
#define ES8388_DACCONTROL3_RAMPRATE_32LRCK	(1 << 6)
#define ES8388_DACCONTROL3_RAMPRATE_64LRCK	(2 << 6)
#define ES8388_DACCONTROL3_RAMPRATE_128LRCK	(3 << 6)

/* R1A/R1B: DAC Digital Volume */
#define ES8388_LDACVOL			0x1A
#define ES8388_RDACVOL			0x1B
#define ES8388_DACVOL_MAX		0xC0

/* R1C: DAC Control 6 */
#define ES8388_DACCONTROL6		0x1C
#define ES8388_DACCONTROL6_CLICKFREE	(1 << 3)

/*
 * R27/R2A: Mixer Controls
 *
 * Bit layout (same for both L/R):
 *   bit7   LD2LO/RD2RO: DAC -> Mixer enable (playback path)
 *   bit6   LI2LO/RI2RO: Line In -> Mixer enable (bypass path)
 *   bit5:3 LI2LOVOL/RI2ROVOL: bypass attenuation (111=0dB ... 000=-15dB)
 *   bit2:0 reserved
 *
 * Default 0xB8 = DAC->Mixer on, bypass off, bypass atten=0dB.
 * Bypass attenuation preset to 0dB so future LI2LO enable won't be silent.
 */
#define ES8388_DACCONTROL17		0x27
#define ES8388_DACCONTROL17_LD2LO	(1 << 7)
#define ES8388_DACCONTROL20		0x2A
#define ES8388_DACCONTROL20_RD2RO	(1 << 7)
#define ES8388_MIXER_DAC_DEFAULT	0xB8

/* R2D: DAC Control 23 */
#define ES8388_DACCONTROL23		0x2D
#define ES8388_DACCONTROL23_VROI_40K	0x10

/* R2E/R2F: Output Volume (0x00=-45dB, 0x1E=0dB, 0x21=+4.5dB max) */
#define ES8388_LOUT1VOL			0x2E
#define ES8388_ROUT1VOL			0x2F
#define ES8388_OUT1VOL_MAX		0x21

/* ========== Private data ========== */

static const char * const es8388_supply_names[ES8388_SUPPLY_NUM] = {
	"DVDD",
	"AVDD",
	"PVDD",
	"HPVDD",
};

struct es8388_priv {
	struct regmap *regmap;
	struct clk *clk;
	struct regulator_bulk_data supplies[ES8388_SUPPLY_NUM];
};

/*
 * reg_defaults: Initial register values written by regcache_sync in probe.
 *
 * These configure chip features that differ from ES8388 IC power-on defaults
 * and are not dynamically managed by bias_level or mute_stream.
 *
 * NOTE: All volume registers MUST be in reg_defaults so that regcache_sync
 * can properly restore user-defined levels after a power cycle.
 *
 * Power registers (R00/R01/R02/R04) are NOT listed here — they are managed
 * exclusively by set_bias_level to ensure correct power sequencing.
 */
static const struct reg_default es8388_reg_defaults[] = {
	{ ES8388_DACCONTROL1, 0x18 },	/* I2S, 16-bit */
	{ ES8388_DACCONTROL2, 0x02 },	/* MCLK/LRCK = 256 */
	{ ES8388_DACCONTROL3, 0x26 },	/* mute=1, soft_ramp=1, 4LRCK/step */
	{ ES8388_DACCONTROL6, ES8388_DACCONTROL6_CLICKFREE },
	{ ES8388_DACCONTROL17, ES8388_MIXER_DAC_DEFAULT },
	{ ES8388_DACCONTROL20, ES8388_MIXER_DAC_DEFAULT },
        { ES8388_LDACVOL, 0xC0 }, // DAC Digital Max
        { ES8388_RDACVOL, 0xC0 },
	{ ES8388_LOUT1VOL, 0x1C },	/* -3dB (headroom for full-scale DAC) */
	{ ES8388_ROUT1VOL, 0x1C },	/* -3dB */
	{ ES8388_DACCONTROL23, ES8388_DACCONTROL23_VROI_40K },
};

static const struct regmap_config es8388_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= ES8388_REG_MAX,
	.cache_type		= REGCACHE_RBTREE,
	.use_single_read	= true,
	.use_single_write	= true,
	.reg_defaults		= es8388_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(es8388_reg_defaults),
};

/* ========== Register access wrappers ========== */

static int es8388_write(struct snd_soc_component *c,
			unsigned int reg, unsigned int val)
{
	int ret = snd_soc_component_write(c, reg, val);

	if (ret < 0)
		dev_err(c->dev, "write reg 0x%02x failed: %d\n", reg, ret);
	return ret;
}

static int es8388_update_bits(struct snd_soc_component *c,
			      unsigned int reg, unsigned int mask,
			      unsigned int val)
{
	int ret = snd_soc_component_update_bits(c, reg, mask, val);

	if (ret < 0)
		dev_err(c->dev, "update reg 0x%02x failed: %d\n", reg, ret);
	return ret < 0 ? ret : 0;
}

/* ========== kcontrols (userspace mixer controls) ========== */

static const DECLARE_TLV_DB_SCALE(dac_tlv, -9600, 50, 0);
static const DECLARE_TLV_DB_SCALE(out_tlv, -4500, 150, 0);

static const struct snd_kcontrol_new es8388_snd_controls[] = {
	SOC_DOUBLE_R_TLV("PCM Volume",
			 ES8388_LDACVOL, ES8388_RDACVOL,
			 0, ES8388_DACVOL_MAX, 1, dac_tlv),
	SOC_DOUBLE_R_TLV("Output 1 Playback Volume",
			 ES8388_LOUT1VOL, ES8388_ROUT1VOL,
			 0, ES8388_OUT1VOL_MAX, 0, out_tlv),
};

/* ========== DAPM (Dynamic Audio Power Management) ========== */

/*
 * Mixer switch controls bound to real registers (R27[7]/R2A[7]).
 * reg_defaults sets initial routing; DAPM reads cache for path discovery;
 * tinymix allows userspace to toggle DAC-to-output routing at runtime.
 */
static const struct snd_kcontrol_new es8388_left_mixer[] = {
	SOC_DAPM_SINGLE("Playback Switch", ES8388_DACCONTROL17, 7, 1, 0),
};

static const struct snd_kcontrol_new es8388_right_mixer[] = {
	SOC_DAPM_SINGLE("Playback Switch", ES8388_DACCONTROL20, 7, 1, 0),
};

static const struct snd_soc_dapm_widget es8388_dapm_widgets[] = {
	/* R02 supply chain: NOPM — bias_level manages R02 directly */
	SND_SOC_DAPM_SUPPLY("DAC STM", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC DIG", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC DLL", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC Vref", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* DAC: NOPM — R04[7:6] managed by bias_level */
	SND_SOC_DAPM_DAC("DACL", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACR", "Playback", SND_SOC_NOPM, 0, 0),

	/* Mixer: NOPM power, real register for switch (R27[7]/R2A[7]) */
	SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
			   es8388_left_mixer, ARRAY_SIZE(es8388_left_mixer)),
	SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
			   es8388_right_mixer, ARRAY_SIZE(es8388_right_mixer)),

	/* Output amp: NOPM — R04[5:2] managed by bias_level */
	SND_SOC_DAPM_PGA("Left Out 1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 2", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Physical output pins */
	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
};

static const struct snd_soc_dapm_route es8388_routes[] = {
	/* R02 supply chain */
	{ "DAC DIG", NULL, "DAC STM" },
	{ "DAC DIG", NULL, "DAC Vref" },
	{ "DAC DIG", NULL, "DAC DLL" },

	{ "DACL", NULL, "DAC DIG" },
	{ "DACR", NULL, "DAC DIG" },

	/* DAC -> Mixer (via Playback Switch) */
	{ "Left Mixer",  "Playback Switch", "DACL" },
	{ "Right Mixer", "Playback Switch", "DACR" },

	/* Mixer -> Output Amp -> Output Pin */
	{ "Left Out 1",  NULL, "Left Mixer" },
	{ "Right Out 1", NULL, "Right Mixer" },
	{ "LOUT1", NULL, "Left Out 1" },
	{ "ROUT1", NULL, "Right Out 1" },
	{ "Left Out 2",  NULL, "Left Mixer" },
	{ "Right Out 2", NULL, "Right Mixer" },
	{ "LOUT2", NULL, "Left Out 2" },
	{ "ROUT2", NULL, "Right Out 2" },
};

/* ========== DAI operations ========== */

static int tom_es8388_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	int ret;

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		ret = es8388_update_bits(component, ES8388_MASTERMODE,
					 ES8388_MASTERMODE_MSC, 0);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ret = es8388_update_bits(component, ES8388_DACCONTROL1,
					 ES8388_DACCONTROL1_DACFMT_MASK, 0);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	return 0;
}

static int tom_es8388_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	int wl, ret;

	switch (params_width(params)) {
	case 16:
		wl = 3;
		break;
	case 18:
		wl = 2;
		break;
	case 20:
		wl = 1;
		break;
	case 24:
		wl = 0;
		break;
	case 32:
		wl = 4;
		break;
	default:
		return -EINVAL;
	}

	ret = es8388_update_bits(component, ES8388_DACCONTROL1,
				 ES8388_DACCONTROL1_DACWL_MASK,
				 wl << ES8388_DACCONTROL1_DACWL_SHIFT);
	if (ret)
		return ret;

	/* Slave mode auto-detects MCLK/LRCK ratio */
	ret = es8388_update_bits(component, ES8388_DACCONTROL2,
				 ES8388_DACCONTROL2_RATEMASK, 0);
	if (ret)
		return ret;

	return 0;
}

static int tom_es8388_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *c = dai->component;

	if (direction != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	/*
	 * Only toggle DAC mute bit (R19[2]). Do NOT touch R1A/R1B here —
	 * those are owned by the "PCM Volume" kcontrol. Writing them from
	 * mute_stream would corrupt the regmap cache and cause tinymix to
	 * show volume=0 while stopped.
	 */
	return es8388_update_bits(c, ES8388_DACCONTROL3,
				  ES8388_DACCONTROL3_DACMUTE,
				  mute ? ES8388_DACCONTROL3_DACMUTE : 0);
}

static const struct snd_soc_dai_ops tom_es8388_dai_ops = {
	.set_fmt	= tom_es8388_set_dai_fmt,
	.hw_params	= tom_es8388_hw_params,
	.mute_stream	= tom_es8388_mute,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver es8388_dai = {
	.name = "tom-es8388-hifi",
	.ops  = &tom_es8388_dai_ops,
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_96000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
};

/* ========== Bias Level (chip-level analog power) ========== */

static int tom_es8388_set_bias_level(struct snd_soc_component *component,
				     enum snd_soc_bias_level level)
{
        struct es8388_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		ret = es8388_write(component, ES8388_CHIPPOWER,
				   ES8388_CHIPPOWER_ALL_ON);
		if (ret)
			return ret;
		ret = es8388_update_bits(component, ES8388_CONTROL1,
				ES8388_CONTROL1_VMIDSEL_MASK |
				ES8388_CONTROL1_ENREF,
				ES8388_CONTROL1_VMIDSEL_50k |
				ES8388_CONTROL1_ENREF);
		if (ret)
			return ret;
		break;

	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_component_get_bias_level(component) ==
		    SND_SOC_BIAS_OFF) {
			/* 5k fast charge for VMID capacitors */
			ret = es8388_update_bits(component, ES8388_CONTROL1,
					ES8388_CONTROL1_VMIDSEL_MASK |
					ES8388_CONTROL1_ENREF,
					ES8388_CONTROL1_VMIDSEL_5k |
					ES8388_CONTROL1_ENREF);
			if (ret)
				return ret;
			msleep(ES8388_VMID_CHARGE_MS);
			/*
			 * Power on DAC + output amp. DAC is muted
			 * (R19[2]=1 from reg_defaults), so no sound leaks.
			 * R04 stays powered until BIAS_OFF (suspend).
			 * TODO: change to 0x3C when LOUT2/ROUT2 hw connected.
			 */
			ret = es8388_write(component, ES8388_DACPOWER,
					   ES8388_DACPOWER_LOUT1_ROUT1);
			if (ret)
				return ret;
			msleep(ES8388_DACPOWER_SETTLE_MS);

                        /*
			 * [STABILITY FIX] Sync regcache here once VMID and DAC power 
			 * are stable. Use priv->regmap directly to avoid undeclared 
			 * function errors.
			 */
			regcache_sync(priv->regmap);
		}

		ret = es8388_write(component, ES8388_CONTROL2,
				   ES8388_CONTROL2_OVERCURRENT |
				   ES8388_CONTROL2_THERMAL);
		if (ret)
			return ret;

		/*
		 * VMID=50k same as PREPARE — avoids bias transient while
		 * output amp is always-on. Trade-off: ~1mA more idle.
		 */
		ret = es8388_update_bits(component, ES8388_CONTROL1,
				ES8388_CONTROL1_VMIDSEL_MASK |
				ES8388_CONTROL1_ENREF,
				ES8388_CONTROL1_VMIDSEL_50k |
				ES8388_CONTROL1_ENREF);
		if (ret)
			return ret;
		break;

	case SND_SOC_BIAS_OFF:
		/* Digital off before analog off */
		ret = es8388_write(component, ES8388_CHIPPOWER,
				   ES8388_CHIPPOWER_ALL_OFF);
		if (ret)
			return ret;
		ret = es8388_write(component, ES8388_DACPOWER,
				   ES8388_DACPOWER_ALL_OFF);
		if (ret)
			return ret;
		ret = es8388_update_bits(component, ES8388_CONTROL1,
				ES8388_CONTROL1_VMIDSEL_MASK |
				ES8388_CONTROL1_ENREF,
				0);
		if (ret)
			return ret;
		break;
	}

	return 0;
}

/* ========== Component lifecycle ========== */

static int es8388_component_probe(struct snd_soc_component *component)
{
	struct es8388_priv *priv = snd_soc_component_get_drvdata(component);
	struct device *dev = component->dev;
	int ret;

	ret = regulator_bulk_enable(ES8388_SUPPLY_NUM, priv->supplies);
	if (ret) {
		dev_err(dev, "unable to enable regulators: %d\n", ret);
		return ret;
	}

	priv->clk = devm_clk_get(dev, "mclk");
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "failed to get mclk: %ld\n", PTR_ERR(priv->clk));
		ret = PTR_ERR(priv->clk);
		goto clk_fail;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "failed to enable mclk: %d\n", ret);
		goto clk_fail;
	}
/* Mark cache as dirty and defer sync to set_bias_level(STANDBY) */
	regcache_mark_dirty(priv->regmap);

        snd_soc_component_force_bias_level(component, SND_SOC_BIAS_STANDBY);

	return 0;

clk_fail:
	regulator_bulk_disable(ES8388_SUPPLY_NUM, priv->supplies);
	return ret;
}

static void es8388_component_remove(struct snd_soc_component *component)
{
	struct es8388_priv *priv = snd_soc_component_get_drvdata(component);

	clk_disable_unprepare(priv->clk);
	regulator_bulk_disable(ES8388_SUPPLY_NUM, priv->supplies);
}

static int es8388_suspend(struct snd_soc_component *component)
{
	struct es8388_priv *priv = snd_soc_component_get_drvdata(component);

	clk_disable_unprepare(priv->clk);
	return regulator_bulk_disable(ES8388_SUPPLY_NUM, priv->supplies);
}

static int es8388_resume(struct snd_soc_component *component)
{
	struct es8388_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(component->dev, "unable to enable clock: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ES8388_SUPPLY_NUM, priv->supplies);
	if (ret) {
		dev_err(component->dev, "unable to enable regulators: %d\n", ret);
		clk_disable_unprepare(priv->clk);
		return ret;
	}

        /* Mark dirty and trigger bias transition to sync registers */
	regcache_mark_dirty(priv->regmap);

        snd_soc_component_force_bias_level(component, SND_SOC_BIAS_STANDBY);

	return 0;
}

/* ========== Component driver ========== */

static const struct snd_soc_component_driver es8388_component_driver = {
	.name			= "tom-es8388",
	.probe			= es8388_component_probe,
	.remove			= es8388_component_remove,
	.suspend		= es8388_suspend,
	.resume			= es8388_resume,
	.set_bias_level		= tom_es8388_set_bias_level,
	.controls		= es8388_snd_controls,
	.num_controls		= ARRAY_SIZE(es8388_snd_controls),
	.dapm_widgets		= es8388_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8388_dapm_widgets),
	.dapm_routes		= es8388_routes,
	.num_dapm_routes	= ARRAY_SIZE(es8388_routes),
	.idle_bias_on		= 1,
	.suspend_bias_off	= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

/* ========== I2C driver ========== */

static int es8388_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct es8388_priv *priv;
	struct regmap *regmap;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(i2c, &es8388_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	priv->regmap = regmap;

	for (i = 0; i < ES8388_SUPPLY_NUM; i++)
		priv->supplies[i].supply = es8388_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ES8388_SUPPLY_NUM, priv->supplies);
	if (ret) {
		dev_err(dev, "unable to get regulators: %d\n", ret);
		return ret;
	}

	dev_set_drvdata(dev, priv);

	return devm_snd_soc_register_component(dev,
				&es8388_component_driver, &es8388_dai, 1);
}

static const struct of_device_id es8388_of_match[] = {
	{ .compatible = "tom,es8388" },
	{ }
};
MODULE_DEVICE_TABLE(of, es8388_of_match);

static struct i2c_driver es8388_driver = {
	.driver = {
		.name		= "tom-es8388",
		.of_match_table	= es8388_of_match,
	},
	.probe_new = es8388_i2c_probe,
};

module_i2c_driver(es8388_driver);

MODULE_DESCRIPTION("Tom ES8388 ALSA SoC audio driver (practice, DAC-only)");
MODULE_AUTHOR("Tom Hsieh <hungen3108@gmail.com>");
MODULE_LICENSE("GPL");

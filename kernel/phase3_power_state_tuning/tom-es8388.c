// SPDX-License-Identifier: GPL-2.0-only
/*
 * tom-es8388.c  --  ES8388 ALSA SoC Audio driver (practice)
 *
 * Author: Tom Hsieh <hungen3108@gmail.com>
 *
 * === Design Principles (matching upstream es8328.c) ===
 *
 * 1. hw_params: ONLY handles PCM-data-related registers
 *    - Word length (R17 DACCONTROL1)
 *    - MCLK/LRCK ratio (R18 DACCONTROL2)
 *    - Does NOT touch power, volume, or routing
 *
 * 2. set_bias_level: Handles chip-level analog power
 *    - R00 (CONTROL1): VMID, EnRef
 *    - R01 (CONTROL2): overcurrent, thermal
 *    - R02 (CHIPPOWER): force all-on in PREPARE as clean baseline
 *
 * 3. DAPM widgets: Handle signal-path power on/off
 *    - R02 bits via SUPPLY widgets (DAC STM/DIG/DLL/Vref)
 *    - R04 bits via DAC/PGA widgets (DACL/R, LOUT1/ROUT1)
 *    - Mixer routing switches (R27[7], R2A[7])
 *
 * 4. kcontrols: Handle user-adjustable parameters
 *    - PCM digital volume (R1A/R1B)
 *    - Output amplifier volume (R2E/R2F)
 *    - Exposed to userspace via ALSA mixer / Android AudioHAL
 *
 * === Why R02=0 in PREPARE? ===
 *
 * DAPM SUPPLY widgets control individual bits of R02 via read-modify-write.
 * During power-up, they execute sequentially:
 *   DAC Vref clears bit0 -> DAC DLL clears bit2 -> DAC STM clears bit4 -> ...
 *
 * Each intermediate state has some power blocks on and others off, which
 * can cause pop noise or unstable behavior. Writing R02=0x00 in PREPARE
 * ensures ALL chip power blocks are enabled atomically before DAPM acts.
 *
 * On the shutdown path (ON->PREPARE->STANDBY), PREPARE also writes R02=0,
 * but DAPM immediately powers down widgets afterward. The brief all-on
 * window is ~1 I2C transaction (~100us), far shorter than the IC's internal
 * settling time, so it causes no audible effect.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

/* ========== Register definitions ========== */

#define ES8388_SUPPLY_NUM	4
#define ES8388_REG_MAX		0x35

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

/* R04: DAC Power Management */
#define ES8388_DACPOWER			0x04
#define ES8388_DACPOWER_RDAC_OFF	6
#define ES8388_DACPOWER_LDAC_OFF	7
#define ES8388_DACPOWER_ROUT1_ON	4
#define ES8388_DACPOWER_LOUT1_ON	5

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
#define ES8388_DACCONTROL3_RAMPRATE_MASK   (3 << 6)
#define ES8388_DACCONTROL3_RAMPRATE_4LRCK  (0 << 6)
#define ES8388_DACCONTROL3_RAMPRATE_32LRCK (1 << 6)
#define ES8388_DACCONTROL3_RAMPRATE_64LRCK (2 << 6)
#define ES8388_DACCONTROL3_RAMPRATE_128LRCK (3 << 6)
#define ES8388_DACCONTROL3_DACSOFTRAMP     (1 << 5)

/* R28: DAC Control 6 */
#define ES8388_DACCONTROL6             0x1C
#define ES8388_DACCONTROL6_CLICKFREE   (1 << 3)

/* DAC Digital Volume */
#define ES8388_LDACVOL			0x1A
#define ES8388_RDACVOL			0x1B
#define ES8388_DACVOL_MAX		0xC0

/* Mixer Controls */
#define ES8388_DACCONTROL17		0x27
#define ES8388_DACCONTROL20		0x2A

/* Output Volume */
#define ES8388_LOUT1VOL			0x2E
#define ES8388_ROUT1VOL			0x2F
#define ES8388_OUT1VOL_MAX		0x24

/* R08: Master Mode */
#define ES8388_MASTERMODE		0x08
#define ES8388_MASTERMODE_MSC		(1 << 7)
#define ES8388_MASTERMODE_MCLKDIV2	(1 << 6)

/* ========== Private data ========== */

struct es8388_priv {
	struct regmap *regmap;
	struct clk *clk;
	struct regulator_bulk_data supplies[ES8388_SUPPLY_NUM];
};

/*
 * reg_defaults: Only registers that are NOT controlled by DAPM or kcontrols.
 *
 * Key change: Removed R27, R2A, R2E, R2F from defaults.
 * - R27[7]/R2A[7] (mixer switches) are controlled by DAPM mixer widgets
 * - R2E/R2F (output volume) are controlled by kcontrols
 * - R1A/R1B (DAC digital volume) are controlled by kcontrols
 * Having them in reg_defaults causes conflicts: regcache_sync can overwrite
 * values that DAPM/kcontrol have already set to a different state.
 */
static const struct reg_default es8388_reg_defaults[] = {
	{ 0x00, 0x05 },  /* CONTROL1: VMIDSEL=500k, EnRef */
	{ 0x01, 0xC0 },  /* CONTROL2: overcurrent + thermal shutdown */
	{ 0x02, 0x00 },  /* CHIPPOWER: all powered on */
	{ 0x04, 0xC0 },  /* DACPOWER: DAC off, outputs off (DAPM controls) */
	{ 0x17, 0x18 },  /* DACCONTROL1: I2S, 16-bit */
	{ 0x18, 0x02 },  /* DACCONTROL2: MCLK/LRCK = 256 */
        { 0x2D, 0x10 },  /* DACCONTROL23: VROI=40k (slow discharge = no stop pop) */
        { ES8388_DACCONTROL3, 0x22 | ES8388_DACCONTROL3_RAMPRATE_128LRCK },
        { ES8388_DACCONTROL6, ES8388_DACCONTROL6_CLICKFREE }, /* 0x08 */
};

static const struct regmap_config es8388_regmap_config = {
	.reg_bits         = 8,
	.val_bits         = 8,
	.max_register     = ES8388_REG_MAX,
	.cache_type       = REGCACHE_RBTREE,
	.use_single_read  = true,
	.use_single_write = true,
	.reg_defaults     = es8388_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es8388_reg_defaults),
};

/* ========== kcontrols (userspace mixer controls) ========== */

/*
 * These are the controls that userspace (Android AudioHAL, alsa_amixer, etc.)
 * uses to adjust volume and settings. Without these, userspace has no way to
 * control the codec's volume, and may silently fail or produce unexpected
 * behavior.
 *
 * TLV (Type-Length-Value) defines the dB scale for each control, so that
 * userspace can display meaningful dB values instead of raw register values.
 */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -9600, 50, 0);   /* R1A/R1B: 0 to -96dB, 0.5dB step */
static const DECLARE_TLV_DB_SCALE(out_tlv, -4500, 150, 0);   /* R2E/R2F: -45 to +4.5dB, 1.5dB step */

static const struct snd_kcontrol_new es8388_snd_controls[] = {
	/* DAC digital volume: R1A (left), R1B (right)
	 * 0x00 = 0dB, 0xC0 = -96dB, inverted = 1 (higher value = lower volume) */
	SOC_DOUBLE_R_TLV("PCM Volume",
			 ES8388_LDACVOL, ES8388_RDACVOL,
			 0, ES8388_DACVOL_MAX, 1, dac_tlv),

	/* Output amplifier volume: R2E (LOUT1), R2F (ROUT1)
	 * 0x00 = -45dB, 0x1E = 0dB, 0x24 = +4.5dB, inverted = 0 */
	SOC_DOUBLE_R_TLV("Output 1 Playback Volume",
			 ES8388_LOUT1VOL, ES8388_ROUT1VOL,
			 0, ES8388_OUT1VOL_MAX, 0, out_tlv),
};

/*
 * Output PGA DAPM event handler for pop-noise suppression.
 *
 * This callback is used to mute/unmute the DAC at proper timing
 * when the output amplifier is powered on/off by DAPM.
 *
 * Power-up sequence (PRE_PMU -> POST_PMU):
 *
 *   1. PRE_PMU:
 *      Mute DAC before enabling output stage.
 *      This prevents unstable analog bias / VMID charging noise
 *      from being amplified by the output PGA.
 *
 *   2. POST_PMU:
 *      Wait for analog circuits (DAC, VMID, output driver)
 *      to settle after power-up.
 *      Then unmute DAC to start playback cleanly.
 *
 * Power-down sequence (PRE_PMD):
 *
 *   3. PRE_PMD:
 *      Mute DAC before disabling output stage.
 *      This avoids pop/click caused by sudden discharge of
 *      output capacitors and bias collapse.
 *
 * Overall goal:
 *   Ensure that audio signal is always muted during
 *   unstable analog power transitions, so that
 *   power on/off does not generate audible pop noise.
 */

static int es8388_out1_pga_event(struct snd_soc_dapm_widget *w,
                                 struct snd_kcontrol *kcontrol,
                                 int event)
{
    struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);

    switch (event) {
    case SND_SOC_DAPM_PRE_PMU:
        /* Mute DAC before output amplifier turns on */
        snd_soc_component_update_bits(c, ES8388_DACCONTROL3,
                                      ES8388_DACCONTROL3_DACMUTE,
                                      ES8388_DACCONTROL3_DACMUTE);
        break;

    case SND_SOC_DAPM_POST_PMU:
        /* Wait longer to ensure VMID bias is fully stable on the decoupling capacitors */
        msleep(150);
        snd_soc_component_update_bits(c, ES8388_DACCONTROL3,
                                      ES8388_DACCONTROL3_DACMUTE, 0);
        break;

    case SND_SOC_DAPM_PRE_PMD:
         /* Ensure enough time for Digital Soft Ramp to finish before cutting analog power */
        snd_soc_component_update_bits(c, ES8388_DACCONTROL3,
                                      ES8388_DACCONTROL3_DACMUTE,
                                      ES8388_DACCONTROL3_DACMUTE);
        msleep(50);
        break;
    }

    return 0;
}


/* ========== DAPM (Dynamic Audio Power Management) ========== */

/* Mixer switch controls - these go INSIDE the DAPM mixer widgets.
 * DAPM will automatically power the mixer path on/off based on whether
 * there is a complete route from a source (Playback stream) to a sink (LOUT1/ROUT1).
 *
 * Unlike the kcontrols above, these are binary on/off switches for signal routing,
 * not user-adjustable volume knobs. */
static const struct snd_kcontrol_new es8388_left_mixer[] = {
	SOC_DAPM_SINGLE("Playback Switch", ES8388_DACCONTROL17, 7, 1, 0),
};

static const struct snd_kcontrol_new es8388_right_mixer[] = {
	SOC_DAPM_SINGLE("Playback Switch", ES8388_DACCONTROL20, 7, 1, 0),
};

static const struct snd_soc_dapm_widget es8388_dapm_widgets[] = {
	/*
	 * R02 SUPPLY widgets: chip-level power blocks.
	 * These establish DAPM dependencies so that the DAC digital block
	 * won't power up until its prerequisites (STM, Vref, DLL) are ready.
	 *
	 * Note: set_bias_level(PREPARE) writes R02=0x00 BEFORE these widgets
	 * act, ensuring no intermediate power states. These widgets then
	 * maintain the correct state and handle orderly shutdown.
	 */
	SND_SOC_DAPM_SUPPLY("DAC STM", ES8388_CHIPPOWER,
			    ES8388_CHIPPOWER_DACSTM_RESET, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC DIG", ES8388_CHIPPOWER,
			    ES8388_CHIPPOWER_DACDIG_OFF, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC DLL", ES8388_CHIPPOWER,
			    ES8388_CHIPPOWER_DACDLL_OFF, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC Vref", ES8388_CHIPPOWER,
			    ES8388_CHIPPOWER_DACVREF_OFF, 1, NULL, 0),

	/* R04[7:6]: DAC power - controlled automatically when stream opens/closes */
	SND_SOC_DAPM_DAC("DACL", "Playback", ES8388_DACPOWER,
			 ES8388_DACPOWER_LDAC_OFF, 1),
	SND_SOC_DAPM_DAC("DACR", "Playback", ES8388_DACPOWER,
			 ES8388_DACPOWER_RDAC_OFF, 1),

	/* Mixer: no power register (SND_SOC_NOPM), routing only */
	SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
			   es8388_left_mixer, ARRAY_SIZE(es8388_left_mixer)),
	SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
			   es8388_right_mixer, ARRAY_SIZE(es8388_right_mixer)),

	/* R04[5:4]: Output amplifier power */
	//SND_SOC_DAPM_PGA("Left Out 1", ES8388_DACPOWER,
	//		 ES8388_DACPOWER_LOUT1_ON, 0, NULL, 0),
	//SND_SOC_DAPM_PGA("Right Out 1", ES8388_DACPOWER,
	//		 ES8388_DACPOWER_ROUT1_ON, 0, NULL, 0),
        SND_SOC_DAPM_PGA_E("Left Out 1", ES8388_DACPOWER,
                           ES8388_DACPOWER_LOUT1_ON, 0, NULL, 0,
                           es8388_out1_pga_event,
                           SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
                           SND_SOC_DAPM_PRE_PMD),

        SND_SOC_DAPM_PGA_E("Right Out 1", ES8388_DACPOWER,
                           ES8388_DACPOWER_ROUT1_ON, 0, NULL, 0,
                           es8388_out1_pga_event,
                           SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
                           SND_SOC_DAPM_PRE_PMD),


	/* Physical output pins */
	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
};

static const struct snd_soc_dapm_route es8388_routes[] = {
	/* R02 supply chain: DIG depends on STM + Vref + DLL */
	{ "DAC DIG", NULL, "DAC STM" },
	{ "DAC DIG", NULL, "DAC Vref" },
	{ "DAC DIG", NULL, "DAC DLL" },

	/* DAC depends on the chip-level digital supply */
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
};

/* ========== DAI operations ========== */

static int tom_es8388_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	int ret;

	/* Clock provider/consumer */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		ret = snd_soc_component_update_bits(component, ES8388_MASTERMODE,
						    ES8388_MASTERMODE_MSC, 0);
		if (ret < 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	/* Interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ret = snd_soc_component_update_bits(component, ES8388_DACCONTROL1,
						    ES8388_DACCONTROL1_DACFMT_MASK, 0);
		if (ret < 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	/* Clock inversion */
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	return 0;
}

/*
 * hw_params: ONLY handles PCM-data-related configuration.
 *
 * This callback is invoked when the PCM stream parameters are negotiated.
 * It configures HOW the codec interprets the incoming I2S data, not whether
 * the codec is powered on (that's bias_level + DAPM's job).
 */
static int tom_es8388_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	int wl;
	int ret;

	switch (params_width(params)) {
	case 16: wl = 3; break;
	case 18: wl = 2; break;
	case 20: wl = 1; break;
	case 24: wl = 0; break;
	case 32: wl = 4; break;
	default: return -EINVAL;
	}

	/* R17: DAC word length only (preserve format bits set by set_dai_fmt) */
	ret = snd_soc_component_update_bits(component, ES8388_DACCONTROL1,
					    ES8388_DACCONTROL1_DACWL_MASK,
					    wl << ES8388_DACCONTROL1_DACWL_SHIFT);
	if (ret < 0)
		return ret;

	/* R18: MCLK/LRCK ratio - slave mode auto-detects, write 0 */
	ret = snd_soc_component_update_bits(component, ES8388_DACCONTROL2,
					    ES8388_DACCONTROL2_RATEMASK, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int tom_es8388_mute(struct snd_soc_dai *dai, int mute, int direction)
{
    if (direction != SNDRV_PCM_STREAM_PLAYBACK)
        return 0;

    return snd_soc_component_update_bits(dai->component, ES8388_DACCONTROL3,
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
		.stream_name  = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates	      = SNDRV_PCM_RATE_8000_96000,
		.formats      = (SNDRV_PCM_FMTBIT_S16_LE |
				 SNDRV_PCM_FMTBIT_S24_LE |
				 SNDRV_PCM_FMTBIT_S32_LE),
	},
};

/* ========== Bias Level (chip-level analog power) ========== */

/*
 * Bias level controls the chip's analog reference and VMID.
 * This is separate from the signal-path power managed by DAPM.
 *
 * Transition sequence during playback:
 *   Boot:   -> OFF -> STANDBY (idle_bias_on)
 *   Play:   STANDBY -> PREPARE -> ON
 *   Pause:  ON -> PREPARE -> STANDBY (after use_pmdown_time)
 *   Play:   STANDBY -> PREPARE -> ON
 *
 * R02 is written to 0x00 in PREPARE to provide a deterministic starting
 * point before DAPM widgets act. See file header comment for details.
 */
static int tom_es8388_set_bias_level(struct snd_soc_component *component,
				     enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
                /* Playback state: Set VROI to 1.5k Ohm for proper output driving capability */
                snd_soc_component_update_bits(component, 0x2D, 0x10, 0);
		break;

	case SND_SOC_BIAS_PREPARE:
		/* R02: Force all chip power blocks on atomically.
		 * This provides a clean baseline before DAPM widgets
		 * individually manage their R02 bits. */
		snd_soc_component_write(component, ES8388_CHIPPOWER, 0);

		/* R00: VMID=50k (fast), enable reference */
		snd_soc_component_update_bits(component, ES8388_CONTROL1,
				ES8388_CONTROL1_VMIDSEL_MASK |
				ES8388_CONTROL1_ENREF,
				ES8388_CONTROL1_VMIDSEL_50k |
				ES8388_CONTROL1_ENREF);

                /* Standby state: Set VROI to 40k Ohm high impedance to prevent pop noise from rapid discharge */
                snd_soc_component_update_bits(component, 0x2D, 0x10, 0x10);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_component_get_bias_level(component) ==
		    SND_SOC_BIAS_OFF) {
			/* First power up: 5k fast charge for VMID capacitors */
			snd_soc_component_update_bits(component, ES8388_CONTROL1,
					ES8388_CONTROL1_VMIDSEL_MASK |
					ES8388_CONTROL1_ENREF,
					ES8388_CONTROL1_VMIDSEL_5k |
					ES8388_CONTROL1_ENREF);
			msleep(50);
		}

		/* R01: Enable overcurrent + thermal protection */
		snd_soc_component_write(component, ES8388_CONTROL2,
				ES8388_CONTROL2_OVERCURRENT |
				ES8388_CONTROL2_THERMAL);

		/* R00: VMID=50k (low power standby), keep reference on */
		snd_soc_component_update_bits(component, ES8388_CONTROL1,
				ES8388_CONTROL1_VMIDSEL_MASK |
				ES8388_CONTROL1_ENREF,
				ES8388_CONTROL1_VMIDSEL_50k |
				ES8388_CONTROL1_ENREF);
		break;

	case SND_SOC_BIAS_OFF:
		/* Disable VMID and reference entirely */
                /* Let DAC fully settle before power-down */
                msleep(50);

                /* Turn off DAC + output amp */
                snd_soc_component_write(component, ES8388_DACPOWER, 0xC0);

                /* Disable VMID and reference */
		snd_soc_component_update_bits(component, ES8388_CONTROL1,
				              ES8388_CONTROL1_VMIDSEL_MASK |
				              ES8388_CONTROL1_ENREF,
				              0);
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

        /* Sync our reg_defaults to hardware (R19, R2D differ from IC defaults) */
        regcache_mark_dirty(priv->regmap);
        ret = regcache_sync(priv->regmap);
        if (ret) {
            dev_err(dev, "failed to sync regcache: %d\n", ret);
            goto sync_fail;
        }

	return 0;

sync_fail:
        clk_disable_unprepare(priv->clk);
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
	struct regmap *regmap = dev_get_regmap(component->dev, NULL);
	int ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(component->dev, "unable to enable clock\n");
		return ret;
	}

	ret = regulator_bulk_enable(ES8388_SUPPLY_NUM, priv->supplies);
	if (ret) {
		dev_err(component->dev, "unable to enable regulators\n");
		clk_disable_unprepare(priv->clk);
		return ret;
	}

	regcache_mark_dirty(regmap);
	ret = regcache_sync(regmap);
	if (ret) {
		dev_err(component->dev, "unable to sync regcache: %d\n", ret);
		return ret;
	}

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
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(i2c, &es8388_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	priv->regmap = regmap;

	priv->supplies[0].supply = "DVDD";
	priv->supplies[1].supply = "AVDD";
	priv->supplies[2].supply = "PVDD";
	priv->supplies[3].supply = "HPVDD";

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
		.of_match_table = es8388_of_match,
	},
	.probe_new = es8388_i2c_probe,
};

module_i2c_driver(es8388_driver);

MODULE_DESCRIPTION("Tom ES8388 ALSA SoC audio driver (practice)");
MODULE_AUTHOR("Tom Hsieh <hungen3108@gmail.com>");
MODULE_LICENSE("GPL");

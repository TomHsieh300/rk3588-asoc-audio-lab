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

#define ES8388_SUPPLY_NUM       4
#define ES8388_REG_MAX          0x35
#define ES8328_DACPOWER         0x04
#define ES8328_CHIPPOWER        0x02

/* ES8328_CHIPPOWER (R02) bit definitions */
#define ES8328_CHIPPOWER_DACVREF_OFF    0
#define ES8328_CHIPPOWER_ADCVREF_OFF    1
#define ES8328_CHIPPOWER_DACDLL_OFF     2
#define ES8328_CHIPPOWER_ADCDLL_OFF     3
#define ES8328_CHIPPOWER_DACSTM_RESET   4
#define ES8328_CHIPPOWER_ADCSTM_RESET   5
#define ES8328_CHIPPOWER_DACDIG_OFF     6
#define ES8328_CHIPPOWER_ADCDIG_OFF     7

struct es8388_priv {
	struct regmap *regmap;
	struct regulator_bulk_data supplies[ES8388_SUPPLY_NUM];
        struct clk *clk;
};

/* Default register values to ensure consistent initial state */
static const struct reg_default es8388_reg_defaults[] = {
	{ 0x00, 0x06 },  /* CONTROL1: VMIDSEL=500k, EnRef */
	{ 0x01, 0xC0 },  /* CONTROL2: overcurrent + thermal shutdown */
	{ 0x02, 0x00 },  /* CHIPPOWER: all powered on (DAC Vref, DLL, STM, DIG all enabled) */
	{ 0x04, 0xC0 },  /* DACPOWER: DAC/Output powered down initially (DAPM will control) */
	{ 0x17, 0x18 },  /* DACCONTROL1: I2S, 16-bit */
	{ 0x18, 0x02 },  /* DACCONTROL2: MCLK/LRCK = 256 */
	{ 0x19, 0x04 },  /* DACCONTROL3: DAC muted */
	{ 0x1A, 0x00 },  /* LDACVOL: 0dB */
	{ 0x1B, 0x00 },  /* RDACVOL: 0dB */
	{ 0x27, 0x80 },  /* DACCONTROL17: Left mixer LD2LO on */
	{ 0x2A, 0x80 },  /* DACCONTROL20: Right mixer RD2RO on */
	{ 0x2E, 0x1E },  /* LOUT1VOL: 0dB */
	{ 0x2F, 0x1E },  /* ROUT1VOL: 0dB */
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

static int tom_es8388_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt) {
	pr_info("ES8388: set_dai_fmt entered\n");

        pr_info("tom-es8388: set_dai_fmt fmt=0x%x\n", fmt);

        switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
                case SND_SOC_DAIFMT_CBC_CFC:
                    pr_info("tom-es8388: codec slave (CBC_CFC)\n");
                    break;
                default:
                    return -EINVAL;
                }
                
                switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
                case SND_SOC_DAIFMT_I2S:
                    pr_info("tom-es8388: format I2S\n");
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
        unsigned int wl;
        unsigned int val;

        dev_info(component->dev, "hw_params: rate=%d, format=%d, channels=%d\n",
                 params_rate(params), params_format(params), params_channels(params));

        switch (params_format(params)) {
            case SNDRV_PCM_FORMAT_S16_LE:
                wl = 0x03;
                break;
            case SNDRV_PCM_FORMAT_S24_LE:
                wl = 0x00;
                break;
            case SNDRV_PCM_FORMAT_S32_LE:
                wl = 0x04;
                break;
            default:
                return -EINVAL;
        }

        /* R17: DAC Control 1 - I2S format + word length */
        /* bit[2:1]=00 (I2S), bit[5:3]=wl */
        val = (wl << 3) | 0x00;  /* I2S format */
        snd_soc_component_write(component, 0x17, val);
        dev_info(component->dev, "hw_params: R17=0x%02x (wl=%d)\n", val, wl);

        /* R18: DAC Control 2 - MCLK/LRCK ratio */
        /* 12.288MHz MCLK / 48kHz = 256, DACFsRatio = 00010 */
        snd_soc_component_write(component, 0x18, 0x02);
        dev_info(component->dev, "hw_params: R18=0x02 (256fs)\n");

        return 0;
}

static int tom_es8388_mute(struct snd_soc_dai *dai, int mute, int direction)
{
    struct snd_soc_component *component = dai->component;
    int ret;
    
    dev_info(component->dev, "mute: %d, direction: %d\n", mute, direction);
    
    ret = snd_soc_component_update_bits(component, 0x19, 0x04,
                                        mute ? 0x04 : 0x00);
    
    /* Debug: print register state after mute/unmute */
    if (ret == 0) {
        dev_info(component->dev, "After mute=%d: R02=0x%02x R04=0x%02x R19=0x%02x\n",
                 mute,
                 snd_soc_component_read(component, 0x02),
                 snd_soc_component_read(component, 0x04),
                 snd_soc_component_read(component, 0x19));
    }
    
    return ret;
}

static const struct snd_soc_dai_ops tom_es8388_dai_ops = {
        .set_fmt   = tom_es8388_set_dai_fmt,
        .hw_params = tom_es8388_hw_params,
        .mute_stream   = tom_es8388_mute,
        .no_capture_mute = 1,
};

static struct snd_soc_dai_driver es8388_dai = {
        .name = "tom-es8388-hifi",
        .ops  = &tom_es8388_dai_ops,
        .playback = {
                .stream_name = "Playback",
                .channels_min = 2,
                .channels_max = 2,
                .rates = (SNDRV_PCM_RATE_192000 |
                          SNDRV_PCM_RATE_96000 | \
                          SNDRV_PCM_RATE_88200 | \
                          SNDRV_PCM_RATE_8000_48000),
                .formats = (SNDRV_PCM_FMTBIT_S16_LE |
                            SNDRV_PCM_FMTBIT_S18_3LE | \
                            SNDRV_PCM_FMTBIT_S20_3LE | \
                            SNDRV_PCM_FMTBIT_S24_LE | \
                            SNDRV_PCM_FMTBIT_S32_LE),
        },
};


static int tom_es8388_set_bias_level(struct snd_soc_component *component,
				     enum snd_soc_bias_level level);

static int es8388_component_probe(struct snd_soc_component *component)
{
	struct device *dev = component->dev;
	struct es8388_priv *priv;
	int ret;

	priv = dev_get_drvdata(dev);
	if (!priv || !priv->regmap) {
		dev_err(dev, "component probe: invalid priv or regmap\n");
		return -EINVAL;
	}

	/* Enable regulators - this is where hardware actually powers up */
	ret = regulator_bulk_enable(ES8388_SUPPLY_NUM, priv->supplies);
	if (ret) {
		dev_err(dev, "unable to enable regulators: %d\n", ret);
		return ret;
	}

	/* Get and enable clock */
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

	/* Hardware just powered up, sync reg_defaults to hardware */
	regcache_cache_only(priv->regmap, false);
	regcache_mark_dirty(priv->regmap);
	ret = regcache_sync(priv->regmap);
	if (ret) {
		dev_err(dev, "failed to sync regcache: %d\n", ret);
		goto sync_fail;
	}

	dev_info(dev, "es8388 component probe ok, mclk=%lu Hz, regcache synced\n",
		 clk_get_rate(priv->clk));

	/* Debug: print initial register state after regcache_sync */
	dev_info(dev, "Initial state: R02=0x%02x R04=0x%02x R19=0x%02x R27=0x%02x R2A=0x%02x\n",
		 snd_soc_component_read(component, 0x02),
		 snd_soc_component_read(component, 0x04),
		 snd_soc_component_read(component, 0x19),
		 snd_soc_component_read(component, 0x27),
		 snd_soc_component_read(component, 0x2A));

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

	if (!priv)
		return;

	clk_disable_unprepare(priv->clk);
	regulator_bulk_disable(ES8388_SUPPLY_NUM, priv->supplies);

	dev_info(component->dev, "es8388 component removed\n");
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
		dev_err(component->dev, "unable to enable clock\n");
		return ret;
	}

	ret = regulator_bulk_enable(ES8388_SUPPLY_NUM, priv->supplies);
	if (ret) {
		dev_err(component->dev, "unable to enable regulators\n");
		clk_disable_unprepare(priv->clk);
		return ret;
	}

	/* Sync regcache with hardware after power up */
	regcache_mark_dirty(priv->regmap);
	ret = regcache_sync(priv->regmap);
	if (ret) {
		dev_err(component->dev, "unable to sync regcache: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct snd_kcontrol_new es8388_left_mixer[] = {
    SOC_DAPM_SINGLE("DACL Switch", 0x27, 7, 1, 0),
};

static const struct snd_kcontrol_new es8388_right_mixer[] = {
    SOC_DAPM_SINGLE("DACR Switch", 0x2A, 7, 1, 0),
};

static const struct snd_soc_dapm_widget es8388_dapm_widgets[] = {
    /* Power supplies for R02 (CHIPPOWER) - these control different bits of R02 */
    SND_SOC_DAPM_SUPPLY("DAC STM", ES8328_CHIPPOWER,
                        ES8328_CHIPPOWER_DACSTM_RESET, 1, NULL, 0),
    SND_SOC_DAPM_SUPPLY("DAC DIG", ES8328_CHIPPOWER,
                        ES8328_CHIPPOWER_DACDIG_OFF, 1, NULL, 0),
    SND_SOC_DAPM_SUPPLY("DAC DLL", ES8328_CHIPPOWER,
                        ES8328_CHIPPOWER_DACDLL_OFF, 1, NULL, 0),
    SND_SOC_DAPM_SUPPLY("DAC Vref", ES8328_CHIPPOWER,
                        ES8328_CHIPPOWER_DACVREF_OFF, 1, NULL, 0),

    /* DAC widgets - controlled by R04 (DACPOWER) */
    SND_SOC_DAPM_DAC("DACL", "Playback", ES8328_DACPOWER, 7, 1),
    SND_SOC_DAPM_DAC("DACR", "Playback", ES8328_DACPOWER, 6, 1),

    /* Output amplifiers - controlled by R04 (DACPOWER) */
    SND_SOC_DAPM_PGA("LOUT1 Amp", ES8328_DACPOWER, 5, 0, NULL, 0),
    SND_SOC_DAPM_PGA("ROUT1 Amp", ES8328_DACPOWER, 4, 0, NULL, 0),

    /* Mixer widgets */
    SND_SOC_DAPM_MIXER("Left Mixer",  SND_SOC_NOPM, 0, 0,
                        es8388_left_mixer, ARRAY_SIZE(es8388_left_mixer)),
    SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
                        es8388_right_mixer, ARRAY_SIZE(es8388_right_mixer)),

    /* Output pins */
    SND_SOC_DAPM_OUTPUT("LOUT1"),
    SND_SOC_DAPM_OUTPUT("ROUT1"),
    SND_SOC_DAPM_OUTPUT("LOUT2"),
    SND_SOC_DAPM_OUTPUT("ROUT2"),
};

static const struct snd_soc_dapm_route es8388_routes[] = {
    /* SUPPLY dependencies - DAC DIG needs these supplies */
    { "DAC DIG", NULL, "DAC STM" },
    { "DAC DIG", NULL, "DAC Vref" },
    { "DAC DIG", NULL, "DAC DLL" },

    /* DAC depends on DAC DIG supply */
    { "DACL", NULL, "DAC DIG" },
    { "DACR", NULL, "DAC DIG" },

    /* Mixer routing */
    { "Left Mixer",  "DACL Switch", "DACL" },
    { "Right Mixer", "DACR Switch", "DACR" },

    /* Output amplifier routing */
    { "LOUT1 Amp", NULL, "Left Mixer" },
    { "ROUT1 Amp", NULL, "Right Mixer" },

    /* Output pin routing */
    { "LOUT1", NULL, "LOUT1 Amp" },
    { "ROUT1", NULL, "ROUT1 Amp" },
};

static const char *bias_level_name(enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_OFF:     return "OFF";
	case SND_SOC_BIAS_STANDBY: return "STANDBY";
	case SND_SOC_BIAS_PREPARE: return "PREPARE";
	case SND_SOC_BIAS_ON:      return "ON";
	default:                   return "UNKNOWN";
	}
}

static int tom_es8388_set_bias_level(struct snd_soc_component *component,
                                     enum snd_soc_bias_level level)
{
    enum snd_soc_bias_level old_level = snd_soc_component_get_bias_level(component);

    dev_info(component->dev, "bias: %s -> %s\n",
             bias_level_name(old_level), bias_level_name(level));

    switch (level) {
    case SND_SOC_BIAS_ON:
        break;

    case SND_SOC_BIAS_PREPARE:
        /* VREF, VMID=2x50k, digital enabled
         * NOTE: We don't touch R02 here! DAPM SUPPLY widgets control R02 bits */
        snd_soc_component_update_bits(component, 0x00, 0x07, 0x05);
        break;

    case SND_SOC_BIAS_STANDBY:
        if (old_level == SND_SOC_BIAS_OFF) {
            /* 5kΩ Fast Charge */
            snd_soc_component_update_bits(component, 0x00, 0x07, 0x07);
            msleep(100);  /* Standard charge time */
        }

        /* R01: overcurrent + thermal shutdown */
        snd_soc_component_write(component, 0x01, 0xC0);
        
        /* VREF, VMID=2*500k, digital stopped */
        snd_soc_component_update_bits(component, 0x00, 0x07, 0x06);
        break;

    case SND_SOC_BIAS_OFF:
        /* Turn OFF VREF */
        snd_soc_component_update_bits(component, 0x00, 0x07, 0x00);
        break;
    }

    return 0;
}

static const struct snd_soc_component_driver es8388_component_driver = {
        .name = "tom-es8388",
        .probe = es8388_component_probe,
        .remove = es8388_component_remove,
        .suspend = es8388_suspend,
        .resume = es8388_resume,
        .set_bias_level = tom_es8388_set_bias_level,
        .dapm_widgets = es8388_dapm_widgets,
        .num_dapm_widgets = ARRAY_SIZE(es8388_dapm_widgets),
        .dapm_routes = es8388_routes,
        .num_dapm_routes = ARRAY_SIZE(es8388_routes),
        .idle_bias_on = 1,
        .suspend_bias_off = 1,
        .use_pmdown_time = 1,
        .endianness = 1,
};

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
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	priv->regmap = regmap;

	/* Setup regulator supplies */
	priv->supplies[0].supply = "DVDD";
	priv->supplies[1].supply = "AVDD";
	priv->supplies[2].supply = "PVDD";
	priv->supplies[3].supply = "HPVDD";

	ret = devm_regulator_bulk_get(dev, ES8388_SUPPLY_NUM, priv->supplies);
	if (ret) {
		dev_err(dev, "unable to get regulators: %d\n", ret);
		return ret;
	}

	/* IMPORTANT: Don't enable regulators/clock here!
	 * They will be enabled in component_probe when ASoC framework is ready */

	dev_set_drvdata(dev, priv);

	ret = devm_snd_soc_register_component(dev,
				&es8388_component_driver, &es8388_dai, 1);
	if (ret) {
		dev_err(dev, "failed to register component: %d\n", ret);
		return ret;
	}

	dev_info(dev, "ES8388 i2c probe ok\n");
	return 0;
}

static const struct of_device_id es8388_of_match[] = {
	{ .compatible = "tom,es8388" },
	{ }
};
MODULE_DEVICE_TABLE(of, es8388_of_match);

static struct i2c_driver es8388_driver = {
	.driver = {
		.name           = "tom-es8388",
		.of_match_table = es8388_of_match,
	},
	.probe_new = es8388_i2c_probe,
};

module_i2c_driver(es8388_driver);

MODULE_DESCRIPTION("Tom ES8388 driver (skeleton: regmap + bulk regulators)");
MODULE_AUTHOR("Tom Hsieh <hungen3108@gmail.com>");
MODULE_LICENSE("GPL");

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <sound/core.h> 
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define ES8388_SUPPLY_NUM 4
#define ES8388_REG_MAX    0x35

struct es8388_priv {
	struct regmap *regmap;
	struct regulator_bulk_data supplies[ES8388_SUPPLY_NUM];
};

static const struct regmap_config es8388_regmap_config = {
	.reg_bits         = 8,
	.val_bits         = 8,
	.max_register     = ES8388_REG_MAX,
	.cache_type       = REGCACHE_RBTREE,
	.use_single_read  = true,
	.use_single_write = true,
};


static struct snd_soc_dai_driver es8388_dai = {
        .name = "tom-es8388-hifi",
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

static int es8388_component_probe(struct snd_soc_component *component)
{
        struct es8388_priv *es8388;

        es8388 = snd_soc_component_get_drvdata(component);

	return 0;
}

static const struct snd_soc_component_driver es8388_component_driver = {
        .name = "tom-es8388",
        .probe = es8388_component_probe,    
};

static int es8388_power_on(struct device *dev, struct es8388_priv *priv)
{
	int ret;

	ret = regulator_bulk_enable(ES8388_SUPPLY_NUM, priv->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	/* Optional: give rails time to settle (keep small; adjust if needed) */
	usleep_range(1000, 2000);

	return 0;
}

static void es8388_power_off(void *data)
{
	struct es8388_priv *priv = data;

	regulator_bulk_disable(ES8388_SUPPLY_NUM, priv->supplies);
}

static int es8388_probe_core(struct device *dev, struct es8388_priv *priv)
{
        int ret;

        priv->supplies[0].supply = "DVDD";
        priv->supplies[1].supply = "AVDD";
        priv->supplies[2].supply = "PVDD";
        priv->supplies[3].supply = "HPVDD";

        ret = devm_regulator_bulk_get(dev, ES8388_SUPPLY_NUM, priv->supplies);
        if (ret) {
                dev_err(dev, "unable to get regulators: %d\n", ret);
                return ret;
        }

        ret = es8388_power_on(dev, priv);
        if (ret)
                return ret;

        ret = devm_add_action_or_reset(dev, es8388_power_off, priv);
        if (ret)
                return ret;

        {
                unsigned int val = 0;
                ret = regmap_read(priv->regmap, 0x00, &val);
                if (ret)
                        dev_warn(dev, "regmap_read sanity failed: %d\n", ret);
                else
                        dev_info(dev, "regmap_read sanity ok: reg0x00=0x%02x\n  ", val);
        }

        return devm_snd_soc_register_component(dev,
                &es8388_component_driver, &es8388_dai, 1);
}

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
	i2c_set_clientdata(i2c, priv);

        ret = es8388_probe_core(dev, priv);
        if (ret)
                return ret;

        dev_info(dev, "Tom ES8388 probed (power/regmap ok)\n");
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


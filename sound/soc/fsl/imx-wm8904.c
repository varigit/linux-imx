/*
 * Copyright (C) 2018 Variscite Ltd. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>

#include "fsl_esai.h"
#include "../codecs/wm8904.h"

#define CODEC_CLK_EXTER_OSC   1
#define CODEC_CLK_ESAI_HCKT   2
#define SUPPORT_RATE_NUM    10

#undef pr_info
#define pr_info printk

struct imx_priv {
	struct clk *codec_clk;
	unsigned int mclk_freq;
	struct platform_device *pdev;
	struct platform_device *asrc_pdev;
	u32 asrc_rate;
	u32 asrc_format;
	bool is_codec_master;
};

static struct imx_priv card_priv;

static int imx_wm8904_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	struct device *dev = &priv->pdev->dev;

	u32 dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;

	int ret = 0;
	priv->mclk_freq = clk_get_rate(priv->codec_clk);

	if (priv->is_codec_master) {
		dai_format |= SND_SOC_DAIFMT_CBM_CFM;
	} else {
		dai_format |= SND_SOC_DAIFMT_CBS_CFS;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, ESAI_HCKT_EXTAL,
			priv->mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8904_CLK_MCLK,
				priv->mclk_freq, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "failed to set codec mclk: %d\n", ret);
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}
		snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2,
					       params_physical_width(params));

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops imx_wm8904_ops = {
	.hw_params = imx_wm8904_hw_params,
};

static const struct snd_soc_dapm_widget imx_wm8904_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static struct snd_soc_dai_link imx_wm8904_dai[] = {
	{
		.name = "WM8904",
		.stream_name = "WM8904 PCM",
		.codec_dai_name = "wm8904-hifi",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF,
		.ops = &imx_wm8904_ops,
	},
};

static struct snd_soc_card imx_wm8904_card = {
	.name = "wm8904-audio",
	.dai_link = imx_wm8904_dai,
	.dapm_widgets = imx_wm8904_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(imx_wm8904_dapm_widgets),
	.fully_routed = true,
	.owner = THIS_MODULE,
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int imx_wm8904_probe(struct platform_device *pdev)
{
	struct device_node *esai_np, *codec_np;
	struct device_node *asrc_np = NULL;
	struct platform_device *esai_pdev;
	struct platform_device *asrc_pdev = NULL;
	struct i2c_client *codec_dev;
	struct imx_priv *priv = &card_priv;
	int ret;

	priv->pdev = pdev;
	priv->asrc_pdev = NULL;

	esai_np = of_parse_phandle(pdev->dev.of_node, "esai-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!esai_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	asrc_np = of_parse_phandle(pdev->dev.of_node, "asrc-controller", 0);
	if (asrc_np) {
		asrc_pdev = of_find_device_by_node(asrc_np);
		priv->asrc_pdev = asrc_pdev;
	}

	esai_pdev = of_find_device_by_node(esai_np);
	if (!esai_pdev) {
		dev_err(&pdev->dev, "failed to find ESAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	priv->is_codec_master = false;
	if (of_property_read_bool(pdev->dev.of_node, "codec-master"))
		priv->is_codec_master = true;

	/*if there is no asrc controller, we only enable one device*/
	imx_wm8904_dai[0].codec_of_node = codec_np;
	imx_wm8904_dai[0].cpu_dai_name = dev_name(&esai_pdev->dev);
	imx_wm8904_dai[0].platform_of_node = esai_np;
	if (priv->is_codec_master)
		imx_wm8904_dai[0].dai_fmt |= SND_SOC_DAIFMT_CBM_CFM;
	else
		imx_wm8904_dai[0].dai_fmt |= SND_SOC_DAIFMT_CBS_CFS;
	imx_wm8904_card.num_links = 1;


	priv->codec_clk = devm_clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(priv->codec_clk)) {
		ret = PTR_ERR(priv->codec_clk);
		dev_err(&codec_dev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	}

	imx_wm8904_card.dev = &pdev->dev;

	ret = snd_soc_of_parse_card_name(&imx_wm8904_card, "model");
	if (ret)
		goto fail;
	ret = snd_soc_of_parse_audio_routing(&imx_wm8904_card, "audio-routing");
	if (ret)
		goto fail;

	platform_set_drvdata(pdev, &imx_wm8904_card);

	ret = snd_soc_register_card(&imx_wm8904_card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
fail:
	if (asrc_np)
		of_node_put(asrc_np);
	if (esai_np)
		of_node_put(esai_np);
	if (codec_np)
		of_node_put(codec_np);
	return ret;
}

static int imx_wm8904_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&imx_wm8904_card);
	return 0;
}

static const struct of_device_id imx_wm8904_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-var-wm8904", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8904_dt_ids);

static struct platform_driver imx_wm8904_driver = {
	.probe = imx_wm8904_probe,
	.remove = imx_wm8904_remove,
	.driver = {
		.name = "imx-wm8904",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8904_dt_ids,
	},
};
module_platform_driver(imx_wm8904_driver);

MODULE_AUTHOR("Variscite Ltd");
MODULE_DESCRIPTION("ALSA SoC wm8904 Machine Layer Driver");
MODULE_ALIAS("platform:imx-wm8904");
MODULE_LICENSE("GPL");

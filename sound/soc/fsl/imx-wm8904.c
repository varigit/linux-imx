/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include "../codecs/wm8904.h"

#undef __USE_LATE_PROBE__
#undef __USE_WM8904_FLL__

struct imx_priv {
	struct platform_device *pdev;
	struct snd_soc_card card;
	struct clk *codec_clk;
	unsigned int clk_frequency;
};

static const struct snd_soc_dapm_widget imx_wm8904_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

#ifdef __USE_LATE_PROBE__
static int imx_wm8904_late_probe(struct snd_soc_card *card)
{
	struct imx_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = clk_prepare_enable(priv->codec_clk);
	if (ret) {
		dev_err(card->dev, "Failed to enable MCLK: %d\n", ret);
		return ret;
	}

	return 0;
}
#endif

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	struct imx_priv *priv = snd_soc_card_get_drvdata(card);
	unsigned int fmt;
	int ret = 0;

	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

	priv->clk_frequency = clk_get_rate(priv->codec_clk);
#ifndef __USE_WM8904_FLL__
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8904_CLK_MCLK,
					priv->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(card->dev, "failed to set codec sysclk: %d\n", ret);
		return ret;
	}
#else
	ret = snd_soc_dai_set_pll(codec_dai, WM8904_FLL_MCLK, WM8904_FLL_MCLK,
		priv->clk_frequency, params_rate(params) * 256);
	if (ret < 0) {
		pr_err("%s - failed to set wm8904 codec PLL.", __func__);
		return ret;
	}

	/*
	 * As here wm8904 use FLL output as its system clock
	 * so calling set_sysclk won't care freq parameter
	 * then we pass 0
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8904_CLK_FLL,
			0, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("%s -failed to set wm8904 SYSCLK\n", __func__);
		return ret;
	}

#endif
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}
/*
	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2,
					params_physical_width(params));
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}
*/
	return ret;
}

#ifndef __USE_LATE_PROBE__
static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_priv *priv = snd_soc_card_get_drvdata(card);
	int ret = 0;

	ret = clk_prepare_enable(priv->codec_clk);
	if (ret) {
		dev_err(card->dev, "Failed to enable MCLK: %d\n", ret);
		return ret;
	}

	return ret;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_priv *priv = snd_soc_card_get_drvdata(card);

	clk_disable_unprepare(priv->codec_clk);
}
#endif

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params = imx_hifi_hw_params,
#ifndef __USE_LATE_PROBE__
	.startup   = imx_hifi_startup,
	.shutdown  = imx_hifi_shutdown,
#endif
};

static struct snd_soc_dai_link imx_wm8904_dai[] = {
	{
		.name = "HiFi",
		.stream_name = "HiFi",
		.codec_dai_name = "wm8904-hifi",
		.ops = &imx_hifi_ops,
	},
};

static int imx_wm8904_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np = NULL;
	struct platform_device *cpu_pdev;
	struct imx_priv *priv;
	struct i2c_client *codec_pdev;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_pdev = of_find_i2c_device_by_node(codec_np);
	if (!codec_pdev || !codec_pdev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	priv->codec_clk = devm_clk_get(&codec_pdev->dev, "mclk");
	if (IS_ERR(priv->codec_clk)) {
		ret = PTR_ERR(priv->codec_clk);
		dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	}

	priv->card.dai_link = imx_wm8904_dai;

	imx_wm8904_dai[0].codec_of_node	= codec_np;
	imx_wm8904_dai[0].cpu_dai_name = dev_name(&cpu_pdev->dev);
	imx_wm8904_dai[0].platform_of_node = cpu_np;
	imx_wm8904_dai[0].dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM;

#ifdef __USE_LATE_PROBE__
	priv->card.late_probe = imx_wm8904_late_probe;
#endif
	priv->card.num_links = 1;
	priv->card.dev = &pdev->dev;
	priv->card.owner = THIS_MODULE;
	priv->card.dapm_widgets = imx_wm8904_dapm_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8904_dapm_widgets);
	priv->card.fully_routed = true,

	ret = snd_soc_of_parse_card_name(&priv->card, "model");
	if (ret)
		goto fail;

	ret = snd_soc_of_parse_audio_routing(&priv->card, "audio-routing");
	if (ret)
		goto fail;

	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	ret = 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static const struct of_device_id imx_wm8904_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8904", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8904_dt_ids);

static struct platform_driver imx_wm8904_driver = {
	.driver = {
		.name = "imx-wm8904",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8904_dt_ids,
	},
	.probe = imx_wm8904_probe,
};
module_platform_driver(imx_wm8904_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX WM8904 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8904");

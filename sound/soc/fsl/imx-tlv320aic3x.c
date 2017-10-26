/*
 * 2014 Variscite, Ltd. All Rights Reserved.
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * 2012 Variscite, Ltd. All Rights Reserved.
 * Based on imx-wm8962.c
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 *
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
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>

#include "../codecs/tlv320aic3x.h"
#include "imx-audmux.h"

#define DAI_NAME_SIZE	32

struct imx_tlv320aic3x_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *codec_clk;
	unsigned int clk_frequency;
	unsigned int output_enable_delay;
	struct gpio_desc *output_enable_gpio;
	struct delayed_work output_enable_work;
};

static void imx_tlv320aic3x_output_enable(struct work_struct *work)
{
	struct imx_tlv320aic3x_data *data = container_of(work,
			struct imx_tlv320aic3x_data, output_enable_work.work);

	if (data->output_enable_gpio)
		gpiod_set_value(data->output_enable_gpio, 1);
}

static int imx_tlv320aic3x_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_tlv320aic3x_data *data = container_of(rtd->card,
					struct imx_tlv320aic3x_data, card);

	if (data->output_enable_gpio)
		queue_delayed_work(system_power_efficient_wq,
			&data->output_enable_work, data->output_enable_delay);

	return 0;
}

static int imx_tlv320aic3x_hwfree(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_tlv320aic3x_data *data = container_of(rtd->card,
					struct imx_tlv320aic3x_data, card);

	if (data->output_enable_gpio)
		gpiod_set_value(data->output_enable_gpio, 0);

	return 0;
}

static struct snd_soc_ops imx_tlv320aic3x_ops = {
	.prepare = imx_tlv320aic3x_prepare,
	.hw_free = imx_tlv320aic3x_hwfree,
};

static int imx_tlv320aic3x_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct imx_tlv320aic3x_data *data = container_of(rtd->card,
					struct imx_tlv320aic3x_data, card);
	struct device *dev = rtd->card->dev;
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0,
				     data->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params\n");
		return ret;
	}

	return 0;
}

static const struct snd_soc_dapm_widget imx_tlv320aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Line Out Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static int imx_tlv320aic3x_audmux_config(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int int_port, ext_port;
	int ret;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;
	ret = imx_audmux_v2_configure_port(int_port,
			IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TFSDIR |
			IMX_AUDMUX_V2_PTCR_TCLKDIR,
			IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}
	ret = imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_SYN,
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

	return 0;
}

static int imx_tlv320aic3x_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np;
	struct platform_device *cpu_pdev;
	struct i2c_client *codec_dev;
	struct imx_tlv320aic3x_data *data;
	struct gpio_desc *gpio;
	int ret;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!cpu_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	if (strstr(cpu_np->name, "ssi")) {
		ret = imx_tlv320aic3x_audmux_config(pdev);
		if (ret)
			goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		return -EINVAL;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->codec_clk = clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(data->codec_clk)) {
		/* assuming clock enabled by default */
		data->codec_clk = NULL;
		ret = of_property_read_u32(codec_np, "clock-frequency",
					&data->clk_frequency);
		if (ret) {
			dev_err(&codec_dev->dev,
				"clock-frequency missing or invalid\n");
			goto fail;
		}
	} else {
		data->clk_frequency = clk_get_rate(data->codec_clk);
		clk_prepare_enable(data->codec_clk);
	}

	gpio = devm_gpiod_get(&pdev->dev, "output-enable", GPIOD_OUT_LOW);
	if (!IS_ERR(gpio)) {
		data->output_enable_gpio = gpio;
		of_property_read_u32(pdev->dev.of_node, "output-enable-delay",
			&ret);
		/* convert output-enable-delay from milliseconds to jiffies */
		data->output_enable_delay = msecs_to_jiffies(ret);
		INIT_DELAYED_WORK(&data->output_enable_work,
					imx_tlv320aic3x_output_enable);
	}

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codec_dai_name = "tlv320aic3x-hifi";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_of_node = cpu_np;
	data->dai.platform_of_node = cpu_np;
	data->dai.init = &imx_tlv320aic3x_dai_init;
	data->dai.ops = &imx_tlv320aic3x_ops;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto clk_fail;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto clk_fail;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_tlv320aic3x_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_tlv320aic3x_dapm_widgets);

	ret = snd_soc_register_card(&data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto clk_fail;
	}

	platform_set_drvdata(pdev, data);
clk_fail:
	clk_put(data->codec_clk);
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_tlv320aic3x_remove(struct platform_device *pdev)
{
	struct imx_tlv320aic3x_data *data = platform_get_drvdata(pdev);

	if (data->codec_clk) {
		clk_disable_unprepare(data->codec_clk);
		clk_put(data->codec_clk);
	}
	snd_soc_unregister_card(&data->card);

	return 0;
}

static const struct of_device_id imx_tlv320aic3x_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-tlv320aic3x", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tlv320aic3x_dt_ids);

static struct platform_driver imx_tlv320aic3x_driver = {
	.driver = {
		.name = "imx-tlv320aic3x",
		.owner = THIS_MODULE,
		.of_match_table = imx_tlv320aic3x_dt_ids,
	},
	.probe = imx_tlv320aic3x_probe,
	.remove = imx_tlv320aic3x_remove,
};
module_platform_driver(imx_tlv320aic3x_driver);

MODULE_AUTHOR("ron.d@variscite.com. Variscite Ltd.");
MODULE_DESCRIPTION("Variscite i.MX TLV320aic3x ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-tlv320aic3x");

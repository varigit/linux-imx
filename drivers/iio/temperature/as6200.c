/*
 * Driver for ams AS6200 temperature sensor.
 *
 * Sensor supports following 7-bit I2C addresses: 0x48, 0x49, 0x4A, 0x4B
 *
 * Copyright (c) 2016 ams AG. All rights reserved.
 *
 * Author: Florian Lobmaier <florian.lobmaier@ams.com>
 * Author: Elitsa Polizoeva <elitsa.polizoeva@ams.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/regmap.h>

static const int as6200_conv_rates[4][2] = { {4, 0}, {1, 0},
					{0, 250000}, {0, 125000} };

/* AS6200 registers */
#define AS6200_REG_TVAL		0x00
#define AS6200_REG_CONFIG	0x01
#define AS6200_REG_TLOW		0x02
#define AS6200_REG_THIGH	0x03
#define AS6200_MAX_REGISTER	0x03

#define AS6200_CONFIG_AL_MASK	BIT(5)
#define AS6200_CONFIG_AL_SHIFT	5
#define AS6200_CONFIG_CR_MASK	GENMASK(7, 6)
#define AS6200_CONFIG_CR_SHIFT	6
#define AS6200_CONFIG_SM_MASK   BIT(8)
#define AS6200_CONFIG_SM_SHIFT  8
#define AS6200_CONFIG_IM_MASK	BIT(9)
#define AS6200_CONFIG_IM_SHIFT	9
#define AS6200_CONFIG_POL_MASK	BIT(10)
#define AS6200_CONFIG_POL_SHIFT	10
#define AS6200_CONFIG_CF_MASK	GENMASK(12, 11)
#define AS6200_CONFIG_CF_SHIFT	11

/* AS6200 init configuration values */
#define AS6200_CONFIG_INIT_IM		0x0
#define AS6200_CONFIG_INIT_POL	0x0
#define AS6200_CONFIG_INIT_CF		0x2

struct as6200_data {
	struct i2c_client *client;
	struct regmap *regmap;
};

static const struct regmap_range as6200_readable_ranges[] = {
	regmap_reg_range(AS6200_REG_TVAL, AS6200_REG_THIGH),
};

static const struct regmap_access_table as6200_readable_table = {
	.yes_ranges = as6200_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(as6200_readable_ranges),
};

static const struct regmap_range as6200_writable_ranges[] = {
	regmap_reg_range(AS6200_REG_CONFIG, AS6200_REG_THIGH),
};

static const struct regmap_access_table as6200_writable_table = {
	.yes_ranges = as6200_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(as6200_writable_ranges),
};

static const struct regmap_range as6200_volatile_ranges[] = {
	regmap_reg_range(AS6200_REG_TVAL, AS6200_REG_TVAL),
};

static const struct regmap_access_table as6200_volatile_table = {
	.yes_ranges = as6200_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(as6200_volatile_ranges),
};

static const struct regmap_config as6200_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = AS6200_MAX_REGISTER,
	.cache_type = REGCACHE_RBTREE,
	.rd_table = &as6200_readable_table,
	.wr_table = &as6200_writable_table,
	.volatile_table = &as6200_volatile_table,
};

static int as6200_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *channel, int *val,
			    int *val2, long mask)
{
	struct as6200_data *data = iio_priv(indio_dev);
	unsigned int reg_val;
	int cr, err;
	s32 ret;

	if (channel->type != IIO_TEMP)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		err = regmap_read(data->regmap, AS6200_REG_TVAL,
					&reg_val);
		if (err < 0)
			return err;
		ret = sign_extend32(reg_val, 15) >> 4;
		if (mask == IIO_CHAN_INFO_PROCESSED)
			*val = (ret * 625) / 10000;
		else
			*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 62;
		*val2 = 500000;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		err = regmap_read(data->regmap, AS6200_REG_CONFIG,
					&reg_val);
		if (err < 0)
			return err;
		cr = (reg_val & AS6200_CONFIG_CR_MASK)
			>> AS6200_CONFIG_CR_SHIFT;
		*val = as6200_conv_rates[cr][0];
		*val2 = as6200_conv_rates[cr][1];
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		break;
	}
	return -EINVAL;
}

static int as6200_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int val,
			int val2,
			long mask)
{
	struct as6200_data *data = iio_priv(indio_dev);
	int i;
	unsigned int config_val;
	int err;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(as6200_conv_rates); i++)
		if ((val == as6200_conv_rates[i][0]) &&
			(val2 == as6200_conv_rates[i][1])) {
			err = regmap_read(data->regmap,
				AS6200_REG_CONFIG, &config_val);
			if (err < 0)
				return err;
			config_val &= ~AS6200_CONFIG_CR_MASK;
			config_val |= i << AS6200_CONFIG_CR_SHIFT;

			return regmap_write(data->regmap, AS6200_REG_CONFIG,
							config_val);
		}
	return -EINVAL;
}

static int as6200_read_thresh(struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			enum iio_event_info info,
			int *val, int *val2)
{
	struct as6200_data *data = iio_priv(indio_dev);
	int ret;
	u32 reg_val;

	dev_info(&data->client->dev, "read thresh called\n");
	if (dir == IIO_EV_DIR_RISING)
		ret = regmap_read(data->regmap, AS6200_REG_THIGH, &reg_val);
	else
		ret = regmap_read(data->regmap, AS6200_REG_TLOW, &reg_val);

	*val = sign_extend32(reg_val, 15) >> 4;

	if (ret)
		return ret;
	return IIO_VAL_INT;
}

static int as6200_write_thresh(struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			enum iio_event_info info,
			int val, int val2)
{
	struct as6200_data *data = iio_priv(indio_dev);
	int ret;
	s16 value = val << 4;

	dev_info(&data->client->dev, "write thresh called %d\n", value);
	if (dir == IIO_EV_DIR_RISING)
		ret = regmap_write(data->regmap, AS6200_REG_THIGH, value);
	else
		ret = regmap_write(data->regmap, AS6200_REG_TLOW, value);
	return ret;
}

static irqreturn_t as6200_alert_isr(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;

	iio_push_event(indio_dev,
		IIO_UNMOD_EVENT_CODE(IIO_TEMP,
					0,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_EITHER),
					iio_get_time_ns(indio_dev));
	return IRQ_HANDLED;
}

static int as6200_init(struct iio_dev *indio_dev)
{
	struct as6200_data *data = iio_priv(indio_dev);
	int err;

	err = regmap_update_bits(data->regmap, AS6200_REG_CONFIG,
			AS6200_CONFIG_IM_MASK,
			AS6200_CONFIG_INIT_IM << AS6200_CONFIG_IM_SHIFT);
	if (err < 0)
		return err;
	err = regmap_update_bits(data->regmap, AS6200_REG_CONFIG,
			AS6200_CONFIG_POL_MASK,
			AS6200_CONFIG_INIT_POL << AS6200_CONFIG_POL_SHIFT);
	if (err < 0)
		return err;
	err = regmap_update_bits(data->regmap, AS6200_REG_CONFIG,
			AS6200_CONFIG_CF_MASK,
			AS6200_CONFIG_INIT_CF << AS6200_CONFIG_CF_SHIFT);
	if (err < 0)
		return err;

	return err;
}

static int as6200_setup_irq(struct iio_dev *indio_dev)
{
	int err;
	struct as6200_data *data = iio_priv(indio_dev);
	struct device *dev = &data->client->dev;
	int irq_trig;
	unsigned int reg_val;
	bool pol;

	err = regmap_read(data->regmap, AS6200_REG_CONFIG, &reg_val);
	if (err < 0)
		return err;

	pol = (reg_val & AS6200_CONFIG_POL_MASK) >> AS6200_CONFIG_POL_SHIFT;
	if (pol)
		irq_trig = IRQF_TRIGGER_RISING;
	else
		irq_trig = IRQF_TRIGGER_FALLING;

	err = request_irq(data->client->irq, as6200_alert_isr, irq_trig,
			"as6200", dev);
	if (err)
		dev_err(dev, "error requesting irq %d\n", err);

	return err;
}

static int as6200_sleep(struct as6200_data *data)
{
	return regmap_update_bits(data->regmap, AS6200_REG_CONFIG,
			AS6200_CONFIG_SM_MASK,
			1 << AS6200_CONFIG_SM_SHIFT);
}

static int as6200_wakeup(struct as6200_data *data)
{
	return regmap_update_bits(data->regmap, AS6200_REG_CONFIG,
			AS6200_CONFIG_SM_MASK,
			0 << AS6200_CONFIG_SM_SHIFT);
}

static ssize_t as6200_show_al(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct as6200_data *data = iio_priv(indio_dev);
	unsigned int reg_val;
	int err;
	bool pol, al;

	err = regmap_read(data->regmap, AS6200_REG_CONFIG, &reg_val);
	if (err < 0)
		return err;
	pol = (reg_val & AS6200_CONFIG_POL_MASK) >> AS6200_CONFIG_POL_SHIFT;
	al = (reg_val & AS6200_CONFIG_AL_MASK) >> AS6200_CONFIG_AL_SHIFT;
	if (pol) {
		if (al)
			return sprintf(buf, "on\n");
		else
			return sprintf(buf, "off\n");
	} else {
		if (al)
			return sprintf(buf, "off\n");
		else
			return sprintf(buf, "on\n");
	}
}

static IIO_DEVICE_ATTR(al, S_IRUGO, as6200_show_al, NULL, 0);

static IIO_CONST_ATTR(sampling_frequency_available, "4 1 0.25 0.125");

static struct attribute *as6200_attrs[] = {
	&iio_dev_attr_al.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static struct attribute_group as6200_attr_group = {
	.attrs = as6200_attrs,
};

static const struct iio_event_spec as6200_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}
};

static const struct iio_chan_spec as6200_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.event_spec = as6200_events,
		.num_event_specs = ARRAY_SIZE(as6200_events),
	}
};

static const struct iio_info as6200_info = {
	.read_raw = as6200_read_raw,
	.write_raw = as6200_write_raw,
	.read_event_value = as6200_read_thresh,
	.write_event_value = as6200_write_thresh,
	.attrs = &as6200_attr_group,
	.driver_module = THIS_MODULE,
};

static int as6200_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct as6200_data *data;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	data->regmap = devm_regmap_init_i2c(client, &as6200_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(&client->dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = id->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &as6200_info;
	indio_dev->channels = as6200_channels;
	indio_dev->num_channels = ARRAY_SIZE(as6200_channels);

	ret = as6200_init(indio_dev);
	if (ret < 0)
		return ret;

	ret = as6200_setup_irq(indio_dev);
	if (ret < 0)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto cleanup_irq;

	return 0;

cleanup_irq:
	free_irq(client->irq, &client->dev);

	return ret;
}

static int as6200_i2c_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	if (client->irq > 0)
		free_irq(client->irq, &client->dev);
	return 0;
}

static const struct of_device_id as6200_of_match[] = {
	{ .compatible = "ams,as6200", },
	{},
};
MODULE_DEVICE_TABLE(of, as6200_of_match);

static const struct i2c_device_id as6200_i2c_id[] = {
	{ "as6200", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, as6200_i2c_id);

#ifdef CONFIG_PM_SLEEP
static int as6200_pm_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct as6200_data *data = iio_priv(indio_dev);

	return as6200_sleep(data);
}

static int as6200_pm_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct as6200_data *data = iio_priv(indio_dev);

	return as6200_wakeup(data);
}
#endif

static const struct dev_pm_ops as6200_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(as6200_pm_suspend, as6200_pm_resume)
};

static struct i2c_driver as6200_i2c_driver = {
	.driver = {
		.name = "as6200",
		.owner = THIS_MODULE,
		.of_match_table = as6200_of_match,
		.pm = &as6200_pm_ops,
	},
	.probe = as6200_i2c_probe,
	.remove = as6200_i2c_remove,
	.id_table = as6200_i2c_id,
};

module_i2c_driver(as6200_i2c_driver);

MODULE_DESCRIPTION("ams AS6200 temperature sensor");
MODULE_AUTHOR("Florian Lobmaier <florian.lobmaier@ams.com>");
MODULE_AUTHOR("Elitsa Polizoeva <elitsa.polizoeva@ams.com>");
MODULE_LICENSE("GPL");

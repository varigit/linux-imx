/*
 *
 * Texas Instruments sn65dsi84 DSI to LVDS bridge driver.
 *
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>

static int sn65dsi84_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}

	return ret;
}

static int sn65dsi84_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int sn65dsi84_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;
	
	return sn65dsi84_i2c_write(client, buf, sizeof(buf));
}

static int sn65dsi84_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct property *prop;
	int err;
	int i, size;
	struct device_node *np = client->dev.of_node;
	int addresses[100];
	int values[100];
	int chipid[] = {0x35, 0x38, 0x49, 0x53, 0x44, 0x20, 0x20, 0x20, 0x01};
	char address, value;
	struct gpio_desc *enable_gpio;

	enable_gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_HIGH);
	if (enable_gpio)
		gpiod_set_value_cansleep(enable_gpio, 1);
	
	for (i = 0; i < sizeof(chipid) / sizeof(int); i++) {
		address = (char)i;
		err = sn65dsi84_i2c_read(client, &address, 1, &value, 1);
		if (err < 0) {
			dev_err(&client->dev, "failed to read chip id\n");
			return err;
		}
		if (value != chipid[i]) {
			dev_err(&client->dev, "chip id is not correct\n");
			return err;
		}
	}

	prop = of_find_property(np, "sn65dsi84,addresses", NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	size = prop->length / sizeof(int);

	err = of_property_read_u32_array(np, "sn65dsi84,addresses", addresses, size);
	if (err && (err != -EINVAL)) {
		dev_err(&client->dev, "Unable to read 'sn65dsi84,addresses'\n");
		return err;
	}
	
	prop = of_find_property(np, "sn65dsi84,values", NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	i = prop->length / sizeof(u32);
	if (i != size) {
		dev_err(&client->dev, "invalid 'sn65dsi84,values' length should be same as addresses\n");
		return -EINVAL;
	}

	err = of_property_read_u32_array(np, "sn65dsi84,values", values, i);
	if (err && (err != -EINVAL)) {
		dev_err(&client->dev, "Unable to read 'sn65dsi84,values'\n");
		return err;
	}
	
	for (i = 0; i < size; i++) {
		sn65dsi84_write_reg(client, addresses[i], values[i]);
		if (err < 0) {
			dev_err(&client->dev, "failed to write data to the chip\n");
			return err;
		}
	}


	return 0;
}

static int sn65dsi84_remove(struct i2c_client *client)
{
	struct gpio_desc *enable_gpio;

	enable_gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_LOW);
	if (enable_gpio)
		gpiod_set_value_cansleep(enable_gpio, 0);

	return 0;
}

static const struct i2c_device_id sn65dsi84_id[] = {
	{"sn65dsi84", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, sn65dsi84_id);

static struct of_device_id sn65dsi84_match_table[] = {
	{ .compatible = "ti,sn65dsi84",},
	{ },
};

static struct i2c_driver sn65dsi84_i2c_driver = {
	.probe = sn65dsi84_probe,
	.remove = sn65dsi84_remove,
	.driver = {
		.name = "sn65dsi84",
		.owner = THIS_MODULE,
		.of_match_table = sn65dsi84_match_table,
	},
	.id_table = sn65dsi84_id,
};

static int __init sn65dsi84_init(void)
{
	return i2c_add_driver(&sn65dsi84_i2c_driver);
}

static void __exit sn65dsi84_exit(void)
{
	i2c_del_driver(&sn65dsi84_i2c_driver);
}

module_init(sn65dsi84_init);
module_exit(sn65dsi84_exit);

MODULE_DESCRIPTION("TI SN65DSI84 DSI to LVDS bridge driver");
MODULE_LICENSE("GPL v2");


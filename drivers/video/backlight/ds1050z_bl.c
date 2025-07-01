// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for Maxim DS1050Z
 *
 * Copyright 2025 Variscite Ltd.
 * Author: Natalia Kovalenko <natasha.k@variscite.com>
 */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>

#define FULL_ON				(0x1 << 5)	/* PWM Duty Cycle 100% */
#define SHUTDOWN_MODE		(0x3 << 6)	/* PWM in low-current state */
/* This mode restores the value
 * set before the low current state
 */
#define RECALL_MODE			(0x1 << 7)

#define DEFAULT_BRIGHTNESS	0x10		/* PWM Duty Cycle 50% */
/* Maximum linear PWM level is 31 (bits [4:0] set), which gives ~96.88% duty cycle.
 * An additional level (32) is available when bit [5] is set (FULL_ON mode),
 * providing 100% duty cycle regardless of bits [4:0].
 */
#define MAX_BRIGHTNESS		32
#define PWM_DATA_MASK		0x1f

struct ds1050z_data {
	struct i2c_client *client;
	struct mutex lock;
	int dft_brightness;
};

static int ds1050z_read(struct i2c_client *client, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte(client);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read from address 0x%x\n", client->addr);
		return ret;
	}

	*val = (u8)ret;

	return 0;
}

static int ds1050z_write(struct i2c_client *client, u8 val)
{
	int ret = i2c_smbus_write_byte(client, val);

	if (ret < 0)
		dev_err(&client->dev, "failed to write to address 0x%x\n", client->addr);

	return ret;
}

static int ds1050z_update_pwm(struct i2c_client *client, u8 pwm_data)
{
	struct ds1050z_data *ds1050z = i2c_get_clientdata(client);
	u8 val;
	int ret;

	mutex_lock(&ds1050z->lock);

	ret = ds1050z_read(client, &val);
	if (ret < 0)
		goto out_unlock;

	if ((val & PWM_DATA_MASK) == pwm_data &&
	    (val & SHUTDOWN_MODE) != SHUTDOWN_MODE &&
	    (val & FULL_ON) != FULL_ON)
		goto out_unlock;

	if ((val & SHUTDOWN_MODE) == SHUTDOWN_MODE) {
		ret = ds1050z_write(client, RECALL_MODE);
		if (ret < 0)
			goto out_unlock;
	}

	ret = ds1050z_write(client, pwm_data);

out_unlock:
	mutex_unlock(&ds1050z->lock);
	return ret;
}

static int ds1050z_set_brightness(struct ds1050z_data *ds1050z, int brightness)
{
	int pwm_level;
	int ret;

	if (brightness >= MAX_BRIGHTNESS)
		pwm_level = FULL_ON;
	else if (brightness <= 0)
		pwm_level = SHUTDOWN_MODE;
	else
		pwm_level = brightness;

	ret = ds1050z_update_pwm(ds1050z->client, (u8)pwm_level);
	if (ret < 0) {
		dev_err(&ds1050z->client->dev,
			"Failed to set brightness to %d, ret = %d\n",
			brightness, ret);
	}

	return ret;
}

static int ds1050z_backlight_update_status(struct backlight_device *backlight_dev)
{
	struct ds1050z_data *ds1050z = bl_get_data(backlight_dev);
	int brightness = backlight_get_brightness(backlight_dev);

	return ds1050z_set_brightness(ds1050z, brightness);
}

static const struct backlight_ops ds1050z_backlight_ops = {
	.update_status = ds1050z_backlight_update_status,
};

static void ds1050z_parse_dt(struct ds1050z_data *ds1050z)
{
	u32 val;

	if (!of_property_read_u32(ds1050z->client->dev.of_node, "default-brightness", &val)) {
		if (val > MAX_BRIGHTNESS) {
			dev_warn(&ds1050z->client->dev,
				"invalid default brightness level: %u, using %u\n",
				val, MAX_BRIGHTNESS);
			val = MAX_BRIGHTNESS;
		}
		ds1050z->dft_brightness = (int)val;
	} else {
		ds1050z->dft_brightness = DEFAULT_BRIGHTNESS;
	}
}

static int ds1050z_probe(struct i2c_client *client)
{
	struct backlight_device *backlight_dev;
	struct backlight_properties props;
	struct ds1050z_data *ds1050z;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "I2C adapter does not support SMBUS Byte Data\n");
		return -EIO;
	}

	ds1050z = devm_kzalloc(&client->dev, sizeof(*ds1050z), GFP_KERNEL);
	if (!ds1050z)
		return -ENOMEM;

	ds1050z->client = client;

	ds1050z_parse_dt(ds1050z);

	i2c_set_clientdata(client, ds1050z);

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;
	props.brightness = ds1050z->dft_brightness;

	mutex_init(&ds1050z->lock);
	backlight_dev = devm_backlight_device_register(&client->dev,
		"ds1050z-backlight", &client->dev, ds1050z,
		&ds1050z_backlight_ops, &props);
	if (IS_ERR(backlight_dev)) {
		return dev_err_probe(&client->dev, PTR_ERR(backlight_dev),
			"failed to register backlight device\n");
	}

	ret = backlight_update_status(backlight_dev);
	if (ret < 0)
		dev_warn(&client->dev, "Failed to update backlight status: %d\n", ret);

	return 0;
}

static void ds1050z_remove(struct i2c_client *client)
{
	struct ds1050z_data *ds1050z = i2c_get_clientdata(client);

	ds1050z_write(client, SHUTDOWN_MODE);
}

#ifdef CONFIG_PM_SLEEP
static int ds1050z_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds1050z_data *ds1050z = i2c_get_clientdata(client);

	ds1050z_write(client, SHUTDOWN_MODE);

	return 0;
}

static int ds1050z_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds1050z_data *ds1050z = i2c_get_clientdata(client);

	ds1050z_write(client, RECALL_MODE);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ds1050z_pm_ops, ds1050z_suspend,
			ds1050z_resume);

static const struct i2c_device_id ds1050z_ids[] = {
	{ "ds1050z", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ds1050z_ids);

static const struct of_device_id ds1050z_match_table[] = {
	{
		.compatible = "maxim,ds1050z",
	},
	{},
};

static struct i2c_driver ds1050z_driver = {
	.driver = {
		.name = "ds1050z",
		.pm = &ds1050z_pm_ops,
		.of_match_table = ds1050z_match_table,
	},
	.probe = ds1050z_probe,
	.remove = ds1050z_remove,
	.id_table = ds1050z_ids,
};

module_i2c_driver(ds1050z_driver);

MODULE_DESCRIPTION("Maxim DS1050Z Backlight Driver");
MODULE_AUTHOR("Natalia Kovalenko <natasha.k@variscite.com>");
MODULE_LICENSE("GPL");

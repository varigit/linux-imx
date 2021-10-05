// SPDX-License-Identifier: GPL-2.0

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "basler-camera-driver.h"

#define DRIVER_VERSION	"1.5.3"
#define SENSOR_NAME	"basler-camera"

/*
 * ABRM register offsets
 *
 */
#define ABRM_GENCP_VERSION		0x0
#define ABRM_MANUFACTURER_NAME		0x4
#define ABRM_MODEL_NAME			0x44
#define ABRM_FAMILY_NAME		0x84
#define ABRM_DEVICE_VERSION		0xC4
#define ABRM_MANUFACTURER_INFO		0x104
#define ABRM_SERIAL_NUMBER		0x144
#define ABRM_USER_DEFINED_NAME		0x184
#define ABRM_DEVICE_CAPABILITIES	0x1C4

/*
 * ABRM register bits
 *
 */
#define ABRM_DEVICE_CAPABILITIES_USER_DEFINED_NAMES_SUPPORT	0x1
#define ABRM_DEVICE_CAPABILITIES_STRING_ENCODING		0x0f
#define ABRM_DEVICE_CAPABILITIES_FAMILY_NAME			0x100


/*
 * Maximum read i2c burst
 *
 * TODO: To be replace by a register call of SBRM
 *
 */
#define I2C_MAXIMUM_READ_BURST	8

static int basler_write_register_chunk(struct i2c_client *client, u8 *buffer,
				      u16 buffer_size);
static int basler_read_register_chunk(struct i2c_client *client, u8 *buffer,
				      u16 buffer_size, __be16 register_address);

static int basler_camera_s_ctrl(struct v4l2_ctrl *ctrl);
static int basler_camera_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int basler_camera_validate(const struct v4l2_ctrl *ctrl, u32 idx,
				  union v4l2_ctrl_ptr ptr);
static void basler_camera_init(const struct v4l2_ctrl *ctrl, u32 idx,
			       union v4l2_ctrl_ptr ptr);
static bool basler_camera_equal(const struct v4l2_ctrl *ctrl, u32 idx,
				union v4l2_ctrl_ptr ptr1,
				union v4l2_ctrl_ptr ptr2);

struct basler_camera_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;
	struct v4l2_ctrl_handler ctrl_handler;

	struct basler_device_information device_information;

	/* Storage for register address and data size for register reads */
	struct register_access ra_tmp;
};

/**
 * basler_write_burst - issue a burst I2C message in master transmit mode
 * @client: Handle to slave device
 * @ra_p: Data structure that hold the register address and data that will be
 *	written to the slave
 *
 * Returns the number of bytes written on success or a negative error code.
 */
static int basler_write_burst(struct basler_camera_dev *sensor,
			      struct register_access *ra_p)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	u16 old_address;

	if (ra_p->data_size > sizeof(ra_p->data)) {
		dev_err(&client->dev,
			"i2c burst array too big, max allowed %lu, got %d\n",
			sizeof(ra_p->data), ra_p->data_size);
		return -EINVAL;
	}

	old_address = ra_p->address;
	ra_p->address = cpu_to_be16(ra_p->address);

	if (I2CREAD == (ra_p->command | I2CREAD)) {
		sensor->ra_tmp.address = ra_p->address;
		sensor->ra_tmp.data_size = ra_p->data_size;

		/* restore original address */
		ra_p->address = old_address;

		return ra_p->data_size;
	}

	if (I2CWRITE == (ra_p->command | I2CWRITE)) {
		ret = basler_write_register_chunk(client, (u8 *)ra_p,
				ra_p->data_size + sizeof(ra_p->address));
		if (ret < 0)
			return ret;

		ra_p->data_size = ret;

		/* restore original address */
		ra_p->address = old_address;

		return ret;
	}

	return -EPERM;
}

static int basler_write_register_chunk(struct i2c_client *client, u8 *buffer,
				       u16 buffer_size)
{
	int ret;

	ret = i2c_master_send(client, buffer, buffer_size);
	if (ret < 0)
		return ret;

	if (ret != buffer_size)
		return -EIO;

	return ret;
}

/**
 * basler_read_burst - issue a burst I2C message in master transmit mode
 * @client: Handle to slave device
 * @ra_p: Data structure store the data read from slave
 *
 * Note: Before data can read use basler_write_burst with read command
 *       to send the register address
 *
 * Returns the number of bytes read on success or a negative error code.
 */
static int basler_read_burst(struct basler_camera_dev *sensor,
			     struct register_access *ra_p)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = basler_read_register_chunk(client, ra_p->data,
			sensor->ra_tmp.data_size, sensor->ra_tmp.address);
	if (ret < 0)
		ra_p->data_size = 0;
	else
		ra_p->data_size = ret;

	return ret;
}

static int basler_read_register_chunk(struct i2c_client *client, u8 *buffer,
				      u16 buffer_size, __be16 register_address)
{
	struct i2c_msg msgs[2] = {};
	int ret = 0;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].buf = (u8 *)&register_address;
	msgs[0].len = sizeof(register_address);

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = buffer;
	msgs[1].len = buffer_size;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("i2c_transfer() failed: %d\n", ret);
		return ret;
	}

	if (ret != 2) {
		pr_err("i2c_transfer() incomplete");
		return -EIO;
	}

	return msgs[1].len;
}

static int basler_read_register(struct i2c_client *client, u8 *buffer,
				u8 buffer_size, u16 register_address)
{
	int ret = 0;
	u8 l_read_bytes = 0;

	do {
		__be16 l_register_address =
			cpu_to_be16(register_address + l_read_bytes);

		ret = basler_read_register_chunk(
			client, (u8 *)buffer + l_read_bytes,
			(u16)min(I2C_MAXIMUM_READ_BURST,
				  ((int)buffer_size - l_read_bytes)),
			l_register_address);
		if (ret < 0) {
			pr_err("basler_read_register_chunk() failed: %d\n",
			       ret);
			return ret;
		}

		if (ret == 0) {
			pr_err("basler_read_register_chunk() read 0 bytes.\n");
			return -EIO;
		}

		l_read_bytes = l_read_bytes + ret;
	} while (l_read_bytes < buffer_size);

	return l_read_bytes;
}

static int
basler_retrieve_device_information(struct i2c_client *client,
				   struct basler_device_information *bdi)
{
	int ret = 0;
	u64 deviceCapabilities = 0;
	u64 stringEncoding = 0;
	__be64 deviceCapabilitiesBe = 0;
	__be32 gencpVersionBe = 0;

	bdi->_magic = BDI_MAGIC;

	ret = basler_read_register(client, (u8 *)&deviceCapabilitiesBe,
				   sizeof(deviceCapabilitiesBe),
				   ABRM_DEVICE_CAPABILITIES);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != sizeof(deviceCapabilitiesBe)) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	deviceCapabilities = be64_to_cpu(deviceCapabilitiesBe);
	stringEncoding = (deviceCapabilities &
			  ABRM_DEVICE_CAPABILITIES_STRING_ENCODING) >> 4;
	pr_debug("deviceCapabilities = 0x%llx\n", deviceCapabilities);
	pr_debug("String Encoding = 0x%llx\n", stringEncoding);

	ret = basler_read_register(client, (u8 *)&gencpVersionBe,
				   sizeof(gencpVersionBe), ABRM_GENCP_VERSION);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != sizeof(gencpVersionBe)) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	bdi->gencpVersion = be32_to_cpu(gencpVersionBe);
	pr_debug("l_gencpVersion = %d.%d\n",
		 (bdi->gencpVersion & 0xffff0000) >> 16,
		 bdi->gencpVersion & 0xffff);

	ret = basler_read_register(client, bdi->deviceVersion,
				   GENCP_STRING_BUFFER_SIZE,
				   ABRM_DEVICE_VERSION);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->deviceVersion = %s\n", bdi->deviceVersion);

	ret = basler_read_register(client, bdi->serialNumber,
				   GENCP_STRING_BUFFER_SIZE,
				   ABRM_SERIAL_NUMBER);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->serialNumber = %s\n", bdi->serialNumber);

	ret = basler_read_register(client, bdi->manufacturerName,
				   GENCP_STRING_BUFFER_SIZE,
				   ABRM_MANUFACTURER_NAME);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->manufacturerName = %s\n", bdi->manufacturerName);

	ret = basler_read_register(client, bdi->modelName,
				   GENCP_STRING_BUFFER_SIZE, ABRM_MODEL_NAME);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->modelName = %s\n", bdi->modelName);

	if (deviceCapabilities & ABRM_DEVICE_CAPABILITIES_FAMILY_NAME) {
		ret = basler_read_register(client, bdi->familyName,
					   GENCP_STRING_BUFFER_SIZE,
					   ABRM_FAMILY_NAME);
		if (ret < 0) {
			pr_err("basler_read_register() failed: %d\n", ret);
			return ret;
		}

		if (ret != GENCP_STRING_BUFFER_SIZE) {
			pr_err("basler_read_register() not read full buffer = %d bytes\n",
			       ret);
			return -EIO;
		}

		pr_debug("bdi->familyName = %s\n", bdi->familyName);
	} else
		pr_notice("ABRM FamilyName not supported\n");

	if (deviceCapabilities &
	    ABRM_DEVICE_CAPABILITIES_USER_DEFINED_NAMES_SUPPORT) {
		ret = basler_read_register(client, bdi->userDefinedName,
					   GENCP_STRING_BUFFER_SIZE,
					   ABRM_USER_DEFINED_NAME);
		if (ret < 0) {
			pr_err("basler_read_register() failed: %d\n", ret);
			return ret;
		}

		if (ret != GENCP_STRING_BUFFER_SIZE) {
			pr_err("basler_read_register() not read full buffer = %d bytes\n",
			       ret);
			return -EIO;
		}

		pr_debug("bdi->userDefinedName = %s\n", bdi->userDefinedName);
	} else
		pr_notice("ABRM UserDefinedName not supported\n");

	ret = basler_read_register(client, bdi->manufacturerInfo,
				   GENCP_STRING_BUFFER_SIZE,
				   ABRM_MANUFACTURER_INFO);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->manufacturerInfo = %s\n", bdi->manufacturerInfo);

	/*
	 * If the strings are in ASCII - print it.
	 */
	if (stringEncoding == 0) {
		pr_info("ABRM: Manufactuturer: %s, Model: %s, Device: %s, Serial: %s\n",
			bdi->manufacturerName, bdi->modelName,
			bdi->deviceVersion, bdi->serialNumber);
	}

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0) // since 4.20.0-rc1
static int basler_retrieve_csi_information(struct basler_camera_dev *sensor,
					   struct basler_csi_information *bci)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct device_node *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret = 0;
	unsigned short i;

	/* We need a function that searches for the device that holds
	 * the csi-2 bus information. For now we put the bus information
	 * also into the sensor endpoint itself.
	 */
	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -ENODEV;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep), &bus_cfg);
	of_node_put(ep);
	if (ret) {
		dev_err(dev, "failed to parse endpoint\n");
		goto done;
	}

	if (bus_cfg.bus_type != V4L2_MBUS_CSI2_DPHY ||
	    bus_cfg.bus.mipi_csi2.num_data_lanes == 0 ||
	    bus_cfg.nr_of_link_frequencies == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		ret = -ENODATA;
		goto done;
	}

	bci->max_lane_frequency = bus_cfg.link_frequencies[0];
	bci->lane_count = bus_cfg.bus.mipi_csi2.num_data_lanes;
	for (i = 0; i < bus_cfg.bus.mipi_csi2.num_data_lanes; ++i)
		bci->lane_assignment[i] = bus_cfg.bus.mipi_csi2.data_lanes[i];

done:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}
#else
static int basler_retrieve_csi_information(struct basler_camera_dev *sensor,
					   struct basler_csi_information *bci)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fwnode_endpoint *endpoint;
	struct device_node *ep;
	int ret = 0;
	unsigned short i;

	/* We need a function that searches for the device that holds
	 * the csi-2 bus information. For now we put the bus information
	 * also into the sensor endpoint itself.
	 */
	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -ENODEV;
	}

	endpoint = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep));
	of_node_put(ep);
	if (IS_ERR(endpoint)) {
		dev_err(dev, "failed to parse endpoint\n");
		return PTR_ERR(endpoint);
	}

	if (endpoint->bus_type != V4L2_MBUS_CSI2 ||
	    endpoint->bus.mipi_csi2.num_data_lanes == 0 ||
	    endpoint->nr_of_link_frequencies == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		ret = -ENODATA;
		goto done;
	}

	bci->max_lane_frequency = endpoint->link_frequencies[0];
	bci->lane_count = endpoint->bus.mipi_csi2.num_data_lanes;
	for (i = 0; i < endpoint->bus.mipi_csi2.num_data_lanes; ++i)
		bci->lane_assignment[i] = endpoint->bus.mipi_csi2.data_lanes[i];

done:
	v4l2_fwnode_endpoint_free(endpoint);
	return ret;
}
#endif

static inline struct basler_camera_dev *
to_basler_camera_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct basler_camera_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct basler_camera_dev,
			     ctrl_handler)
			->sd;
}

/**
 * basler_camera_set_fmt - set format of the camera
 *
 * Note: Will be done in user space
 *
 * Returns always zero
 */
static int basler_camera_set_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *format)
{
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	if (format->pad)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	sensor->format = format->format;

	mutex_unlock(&sensor->lock);

	return 0;
}

/**
 * basler_camera_get_fmt - Get current format
 *
 * Returns always zero
 */
static int basler_camera_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	if (format->pad)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	format->format = sensor->format;

	mutex_unlock(&sensor->lock);

	return 0;
}

/**
 * basler_camera_s_stream - start camera streaming
 *
 * Note: Will be done in user space
 *
 * Returns always zero
 */
static int basler_camera_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int basler_camera_s_power(struct v4l2_subdev *sd, int on)
{
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	mutex_lock(&sensor->lock);

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);

	mutex_unlock(&sensor->lock);

	return 0;
}

static long basler_camera_priv_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);
	int ret;

	switch (cmd) {
	case BASLER_IOC_G_INTERFACE_VERSION:
	{
		__u32 __user *uptr = (__u32 __user *)arg;

		__u32 version = (((__u32)BASLER_INTERFACE_VERSION_MAJOR) << 16) | BASLER_INTERFACE_VERSION_MINOR;
		ret = put_user(version, uptr);
		if (ret)
			return -EFAULT;

		return 0;
	}

	case BASLER_IOC_READ_REGISTER:
	{
		struct register_access __user *ra_p = (struct register_access __user *)arg;
		struct register_access ra;
		u8 buf[BASLER_REGISTER_ACCESS_BUF_SIZE];
		__be16 reg_addr;

		ret = copy_from_user(&ra, ra_p, sizeof(struct register_access));
		if (ret)
			return -EFAULT;

		reg_addr = cpu_to_be16(ra.address);

		ret = basler_read_register_chunk(client, buf, ra.data_size, reg_addr);
		if (ret < 0)
			return ret;

		ret = copy_to_user(ra_p->data, buf, ra.data_size);
		if (ret)
			return -EFAULT;

		return 0;
	}

	case BASLER_IOC_WRITE_REGISTER:
	{
		struct register_access __user *ra_p = (struct register_access __user *)arg;
		struct register_access ra;

		ret = copy_from_user(&ra, ra_p, sizeof(struct register_access));
		if (ret)
			return -EFAULT;

		ra.address = cpu_to_be16(ra.address);

		ret = basler_write_register_chunk(client, (u8 *)&ra, ra.data_size + sizeof(ra.address));
		if (ret < 0)
			return ret;

		return 0;
	}

	case BASLER_IOC_G_DEVICE_INFORMATION:
	{
		struct basler_device_information __user *uptr = (struct basler_device_information __user *)arg;

		ret = copy_to_user(uptr, &sensor->device_information, sizeof(struct basler_device_information));
		if (ret)
			return -EFAULT;

		return 0;
	}

	case BASLER_IOC_G_CSI_INFORMATION:
	{
		struct basler_csi_information __user *uptr = (struct basler_csi_information __user *)arg;
		struct basler_csi_information csi_info;

		ret = basler_retrieve_csi_information(sensor, &csi_info);
		if (ret < 0)
			return ret;

		ret = copy_to_user(uptr, &csi_info, sizeof(struct basler_csi_information));
		if (ret)
			return -EFAULT;

		return 0;
	}

	default:
		break;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_core_ops basler_camera_core_ops = {
	.s_power = basler_camera_s_power,
	.ioctl = basler_camera_priv_ioctl,
};

static const struct v4l2_subdev_video_ops basler_camera_video_ops = {
	.s_stream = basler_camera_s_stream,
};

static const struct v4l2_subdev_pad_ops basler_camera_pad_ops = {
	.set_fmt = basler_camera_set_fmt,
	.get_fmt = basler_camera_get_fmt,
};

static const struct v4l2_subdev_ops basler_camera_subdev_ops = {
	.core = &basler_camera_core_ops,
	.video = &basler_camera_video_ops,
	.pad = &basler_camera_pad_ops,
};

static const struct v4l2_ctrl_ops basler_camera_ctrl_ops = {
	.g_volatile_ctrl = basler_camera_g_volatile_ctrl,
	.s_ctrl = basler_camera_s_ctrl,
};

static const struct v4l2_ctrl_type_ops basler_camera_ctrl_type_ops = {
	.validate = basler_camera_validate,
	.init = basler_camera_init,
	.equal = basler_camera_equal,
};

/**
 * basler_camera_validate
 *
 * Note: Not needed by access-register control
 *
 * Returns always zero
 */
static int basler_camera_validate(const struct v4l2_ctrl *ctrl, u32 idx,
				  union v4l2_ctrl_ptr ptr)
{
	return 0;
}

/**
 * basler_camera_init
 *
 * Note: Not needed by access-register control
 *
 */
static void basler_camera_init(const struct v4l2_ctrl *ctrl, u32 idx,
			       union v4l2_ctrl_ptr ptr)
{
}

/**
 * basler_camera_equal
 *
 * Note: Not needed by access-register control
 *
 * Returns always zero
 */
static bool basler_camera_equal(const struct v4l2_ctrl *ctrl, u32 idx,
				union v4l2_ctrl_ptr ptr1,
				union v4l2_ctrl_ptr ptr2)
{
	return 0;
}

/* We need to define a new V4L2_CTRL_TYPE value that does not collide
 * with an existing one.
 *
 * The basic V4L2_CTRL_TYPE_XXX values are defined in linux/videodev2.h
 * as enum v4l2_ctrl_type, but several other headers extend these
 * definitions by adding more values through #defines. There seems to be
 * no central coordination facility to avoid collisions when doing so.
 * Therefore, the best we can do is selecting some obscure random value,
 * cross our fingers and hope for the best.
 */
#define V4L2_CTRL_TYPE_UNIQUE	(0x370de8ca)

static const struct v4l2_ctrl_config ctrl_access_register = {
	.ops = &basler_camera_ctrl_ops,
	.type_ops = &basler_camera_ctrl_type_ops,
	.id = V4L2_CID_BASLER_ACCESS_REGISTER,
	.name = "basler-access-register",
	.type = V4L2_CTRL_TYPE_UNIQUE,
	.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE | V4L2_CTRL_FLAG_VOLATILE,
	.step = 1,
	.dims = { 1 },
	.elem_size = sizeof(struct register_access),
};

static const struct v4l2_ctrl_config ctrl_basler_device_information = {
	.ops = &basler_camera_ctrl_ops,
	.type_ops = &basler_camera_ctrl_type_ops,
	.id = V4L2_CID_BASLER_DEVICE_INFORMATION,
	.name = "basler-device-information",
	.type = V4L2_CTRL_TYPE_UNIQUE,
	.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	.step = 1,
	.dims = { 1 },
	.elem_size = sizeof(struct basler_device_information),
};

static const struct v4l2_ctrl_config ctrl_basler_interface_version = {
	.ops = &basler_camera_ctrl_ops,
	.type_ops = &basler_camera_ctrl_type_ops,
	.id = V4L2_CID_BASLER_INTERFACE_VERSION,
	.name = "basler-interface-version",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
	.min = 0x0,
	.max = 0xffffffff,
	.def = (BASLER_INTERFACE_VERSION_MAJOR << 16) |
	       BASLER_INTERFACE_VERSION_MINOR,
	.step = 1,
};

static const struct v4l2_ctrl_config ctrl_basler_csi_information = {
	.ops = &basler_camera_ctrl_ops,
	.type_ops = &basler_camera_ctrl_type_ops,
	.id = V4L2_CID_BASLER_CSI_INFORMATION,
	.name = "basler-csi-information",
	.type = V4L2_CTRL_TYPE_UNIQUE,
	.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	.step = 1,
	.dims = { 1 },
	.elem_size = sizeof(struct basler_csi_information),
};

static int basler_camera_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);
	int ret;
	struct register_access *fp_ra_new;

	switch (ctrl->id) {
	case V4L2_CID_BASLER_ACCESS_REGISTER:

		if (ctrl->elem_size != sizeof(struct register_access)) {
			dev_err(sd->dev,
				"%s: size mismatch V4L2_CID_BASLER_ACCESS_REGISTER: elem = %u, expected = %zu",
				__func__, ctrl->elem_size, sizeof(struct register_access));
			return -ENOMEM;
		}

		fp_ra_new = (struct register_access *)ctrl->p_new.p;
		ret = basler_write_burst(sensor, fp_ra_new);
		if (ret < 0)
			return -EIO;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int
basler_camera_g_ctrl_access_register(struct device *dev,
				     struct basler_camera_dev *sensor,
				     const struct v4l2_ctrl *ctrl)
{
	struct register_access *fp_ra_new = NULL;
	int ret;

	if (ctrl->elem_size != sizeof(struct register_access)) {
		dev_err(dev,
			"%s: size mismatch V4L2_CID_BASLER_ACCESS_REGISTER: elem = %u, expected = %zu",
			__func__, ctrl->elem_size,
			sizeof(struct register_access));
		return -ENOMEM;
	}

	fp_ra_new = (struct register_access *)ctrl->p_new.p;
	ret = basler_read_burst(sensor, fp_ra_new);
	if (ret < 0)
		return ret;

	if (ret == 0)
		return -EIO;

	return 0;
}

static int
basler_camera_g_ctrl_device_information(struct device *dev,
					struct basler_camera_dev *sensor,
					const struct v4l2_ctrl *ctrl)
{
	struct basler_device_information *l_bdi = NULL;

	if (ctrl->elem_size != sizeof(struct basler_device_information)) {
		dev_err(dev,
			"%s: size mismatch V4L2_CID_BASLER_DEVICE_INFORMATION: elem = %u, expected = %zu",
			__func__, ctrl->elem_size,
			sizeof(struct basler_device_information));
		return -ENOMEM;
	}

	l_bdi = (struct basler_device_information *)ctrl->p_new.p;
	memcpy(l_bdi, &sensor->device_information,
	       sizeof(struct basler_device_information));

	return 0;
}

static int
basler_camera_g_ctrl_csi_information(struct device *dev,
				     struct basler_camera_dev *sensor,
				     const struct v4l2_ctrl *ctrl)
{
	struct basler_csi_information *l_bci = NULL;
	int ret;

	if (ctrl->elem_size != sizeof(struct basler_csi_information)) {
		dev_err(dev,
			"%s: size mismatch V4L2_CID_BASLER_CSI_INFORMATION: elem = %u, expected = %zu",
			__func__, ctrl->elem_size,
			sizeof(struct basler_csi_information));
		return -ENOMEM;
	}

	l_bci = (struct basler_csi_information *)ctrl->p_new.p;
	ret = basler_retrieve_csi_information(sensor, l_bci);
	if (ret < 0)
		return ret;

	return 0;
}

static int basler_camera_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct device *dev = sd->dev;
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	switch (ctrl->id) {
	case V4L2_CID_BASLER_ACCESS_REGISTER:
		return basler_camera_g_ctrl_access_register(dev, sensor, ctrl);
	case V4L2_CID_BASLER_DEVICE_INFORMATION:
		return basler_camera_g_ctrl_device_information(dev, sensor, ctrl);
	case V4L2_CID_BASLER_CSI_INFORMATION:
		return basler_camera_g_ctrl_csi_information(dev, sensor, ctrl);
	default:
		break;
	}

	return -EINVAL;
}

/**
 * basler_camera_link_setup
 *
 * Note: Function is needed by imx8qm
 *
 * Returns always zero
 */
static int basler_camera_link_setup(struct media_entity *entity,
				    const struct media_pad *local,
				    const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations basler_camera_sd_media_ops = {
	.link_setup = basler_camera_link_setup,
};

static int basler_camera_alloc_control(struct v4l2_ctrl_handler *hdl,
				       const struct v4l2_ctrl_config *cfg,
				       struct device *dev)
{
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	ctrl = v4l2_ctrl_new_custom(hdl, cfg, NULL);
	if (ctrl == NULL) {
		dev_err(dev,
			"Registering ctrl %s failed: %d\n",
			cfg->name,
			hdl->error);
		ret = hdl->error;
	}

	return ret;
}

static int basler_camera_init_controls(struct basler_camera_dev *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;

	v4l2_ctrl_handler_init(hdl, 32);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	if (basler_camera_alloc_control(hdl, &ctrl_access_register, &sensor->i2c_client->dev) ||
	    basler_camera_alloc_control(hdl, &ctrl_basler_device_information, &sensor->i2c_client->dev) ||
	    basler_camera_alloc_control(hdl, &ctrl_basler_interface_version, &sensor->i2c_client->dev) ||
	    basler_camera_alloc_control(hdl, &ctrl_basler_csi_information, &sensor->i2c_client->dev)) {
		/* free up all controls allocated so far */
		v4l2_ctrl_handler_free(hdl);
		dev_dbg(sensor->sd.v4l2_dev->dev, "%s: ctrl setup error.\n",
			__func__);
		return hdl->error;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;
}

static int basler_camera_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct basler_camera_dev *sensor;
	int ret;

	dev_dbg(dev, " %s driver start probing\n", SENSOR_NAME);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C_FUNC_I2C not supported\n");
		ret = -ENODEV;
		goto exit;
	}

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	v4l2_i2c_subdev_init(&sensor->sd, client, &basler_camera_subdev_ops);

	ret = basler_retrieve_device_information(client,
						 &sensor->device_information);
	if (ret) {
		dev_dbg(dev,
			"basler_retrieve_device_information() failed: %d\n",
			ret);
		goto exit;
	}

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &basler_camera_sd_media_ops;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_dbg(dev, "media_entity_pads_init() failed: %d\n", ret);
		goto exit;
	}

	mutex_init(&sensor->lock);

	ret = basler_camera_init_controls(sensor);
	if (ret) {
		dev_dbg(dev,
			"basler_camera_init_controls() failed: %d\n",
			ret);
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret) {
		dev_dbg(dev,
			"v4l2_async_register_subdev() failed: %d\n",
			ret);
		goto handler_cleanup;
	}

	dev_dbg(dev, " %s driver probed\n", SENSOR_NAME);
	dev_info(dev, "Basler Camera Driver v" DRIVER_VERSION " loaded successfully\n");
	return 0;

handler_cleanup:
	sensor->sd.ctrl_handler = NULL;
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
exit:
	dev_dbg(dev, "%s failure: %d\n", __func__, ret);
	return ret;
}

static int basler_camera_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	sensor->sd.ctrl_handler = NULL;
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);

	return 0;
}

static const struct i2c_device_id basler_camera_id[] = {
	{ "basler-camera", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, basler_camera_id);

static const struct of_device_id basler_camera_dt_ids[] = {
	{ .compatible = "basler,basler-camera" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, basler_camera_dt_ids);

static struct i2c_driver basler_camera_i2c_driver = {
	.driver = {
			.name = "basler-camera",
			.of_match_table = basler_camera_dt_ids,
		},
	.id_table = basler_camera_id,
	.probe = basler_camera_probe,
	.remove = basler_camera_remove,
};

module_i2c_driver(basler_camera_i2c_driver);

MODULE_DESCRIPTION("Basler camera subdev driver");
MODULE_AUTHOR("Sebastian Suesens <sebastian.suesens@baslerweb.com>");
MODULE_LICENSE("GPL");

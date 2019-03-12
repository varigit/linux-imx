// SPDX-License-Identifier: GPL-2.0
/*
 * Flex Timer Module Quadrature decoder
 *
 * This module implements a driver for decoding the FTM quadrature
 * of ex. a LS1021A
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/swait.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/mutex.h>

#define FTM_SC       0x0 /* Status And Control */
#define FTM_CNT      0x4 /* Counter */
#define FTM_MOD      0x8 /* Modulo */

#define FTM_CNTIN    0x4C /* Counter Initial Value */
#define FTM_STATUS   0x50 /* Capture And Compare Status */
#define FTM_MODE     0x54 /* Features Mode Selection */
#define FTM_SYNC     0x58 /* Synchronization */
#define FTM_OUTINIT  0x5C /* Initial State For Channels Output */
#define FTM_OUTMASK  0x60 /* Output Mask */
#define FTM_COMBINE  0x64 /* Function For Linked Channels */
#define FTM_DEADTIME 0x68 /* Deadtime Insertion Control */
#define FTM_EXTTRIG  0x6C /* FTM External Trigger */
#define FTM_POL      0x70 /* Channels Polarity */
#define FTM_FMS      0x74 /* Fault Mode Status */
#define FTM_FILTER   0x78 /* Input Capture Filter Control */
#define FTM_FLTCTRL  0x7C /* Fault Control */
#define FTM_QDCTRL   0x80 /* Quadrature Decoder Control And Status */
#define FTM_CONF     0x84 /* Configuration */
#define FTM_FLTPOL   0x88 /* FTM Fault Input Polarity */
#define FTM_SYNCONF  0x8C /* Synchronization Configuration */
#define FTM_INVCTRL  0x90 /* FTM Inverting Control */
#define FTM_SWOCTRL  0x94 /* FTM Software Output Control */
#define FTM_PWMLOAD  0x98 /* FTM PWM Load */

#define FTM_SC_CLK_MASK_SHIFT	3
#define FTM_SC_CLK_MASK		(3 << FTM_SC_CLK_MASK_SHIFT)
#define FTM_SC_TOF		0x80
#define FTM_SC_TOIE		0x40
#define FTM_SC_CPWMS		0x20
#define FTM_SC_CLKS		0x18
#define FTM_SC_PS_1		0x0
#define FTM_SC_PS_2		0x1
#define FTM_SC_PS_4		0x2
#define FTM_SC_PS_8		0x3
#define FTM_SC_PS_16		0x4
#define FTM_SC_PS_32		0x5
#define FTM_SC_PS_64		0x6
#define FTM_SC_PS_128		0x7
#define FTM_SC_PS_MASK		0x7

#define FTM_MODE_FAULTIE	0x80
#define FTM_MODE_FAULTM		0x60
#define FTM_MODE_CAPTEST	0x10
#define FTM_MODE_PWMSYNC	0x8
#define FTM_MODE_WPDIS		0x4
#define FTM_MODE_INIT		0x2
#define FTM_MODE_FTMEN		0x1

/* NXP Errata: The PHAFLTREN and PHBFLTREN bits are tide to zero internally
 * and these bits cannot be set. Flextimer cannot use Filter in
 * Quadrature Decoder Mode.
 * https://community.nxp.com/thread/467648#comment-1010319
 */
#define FTM_QDCTRL_PHAFLTREN	0x80
#define FTM_QDCTRL_PHBFLTREN	0x40
#define FTM_QDCTRL_PHAPOL	0x20
#define FTM_QDCTRL_PHBPOL	0x10
#define FTM_QDCTRL_QUADMODE	0x8
#define FTM_QDCTRL_QUADDIR	0x4
#define FTM_QDCTRL_TOFDIR	0x2
#define FTM_QDCTRL_QUADEN	0x1

#define FTM_FMS_FAULTF		0x80
#define FTM_FMS_WPEN		0x40
#define FTM_FMS_FAULTIN		0x10
#define FTM_FMS_FAULTF3		0x8
#define FTM_FMS_FAULTF2		0x4
#define FTM_FMS_FAULTF1		0x2
#define FTM_FMS_FAULTF0		0x1

#define FTM_CSC_BASE		0xC
#define FTM_CSC_MSB		0x20
#define FTM_CSC_MSA		0x10
#define FTM_CSC_ELSB		0x8
#define FTM_CSC_ELSA		0x4
#define FTM_CSC(_channel)	(FTM_CSC_BASE + ((_channel) * 8))

#define FTM_CV_BASE		0x10
#define FTM_CV(_channel)	(FTM_CV_BASE + ((_channel) * 8))
struct ftm_quaddec {
	struct platform_device *pdev;
	struct delayed_work delayedcounterwork;
	void __iomem *ftm_base;
	bool big_endian;

	/* Offset added to the counter to adjust for overflows of the
	 * 16 bit HW counter. Only the 16 MSB are set.
	 */
	uint32_t counteroffset;

	/* Store the counter on each read, this is used to detect
	 * if the counter readout if we over or underflow
	 */
	uint8_t lastregion;

	/* Poll-interval, in ms before delayed work must poll counter */
	uint16_t poll_interval;
	
	struct clk *ipg_clk;

	struct mutex ftm_quaddec_mutex;
};

struct counter_result {
	/* 16 MSB are from the counteroffset
	 * 16 LSB are from the hardware counter
	 */
	uint32_t value;
};

#define HASFLAGS(flag, bits) ((flag & bits) ? 1 : 0)

#define DEFAULT_POLL_INTERVAL    100 /* in msec */

static void ftm_read(struct ftm_quaddec *ftm, uint32_t offset, uint32_t *data)
{
	if (ftm->big_endian)
		*data = ioread32be(ftm->ftm_base + offset);
	else
		*data = ioread32(ftm->ftm_base + offset);
}

static void ftm_write(struct ftm_quaddec *ftm, uint32_t offset, uint32_t data)
{
	if (ftm->big_endian)
		iowrite32be(data, ftm->ftm_base + offset);
	else
		iowrite32(data, ftm->ftm_base + offset);
}

/* take mutex
 * call ftm_clear_write_protection
 * update settings
 * call ftm_set_write_protection
 * release mutex
 */
static void ftm_clear_write_protection(struct ftm_quaddec *ftm)
{
	uint32_t flag;

	/* First see if it is enabled */
	ftm_read(ftm, FTM_FMS, &flag);

	if (flag & FTM_FMS_WPEN) {
		ftm_read(ftm, FTM_MODE, &flag);
		ftm_write(ftm, FTM_MODE, flag | FTM_MODE_WPDIS);
	}
}

static void ftm_set_write_protection(struct ftm_quaddec *ftm)
{
	ftm_write(ftm, FTM_FMS, FTM_FMS_WPEN);
}

/* must be called with mutex locked */
static void ftm_work_reschedule(struct ftm_quaddec *ftm)
{
	cancel_delayed_work(&ftm->delayedcounterwork);
	if (ftm->poll_interval > 0)
		schedule_delayed_work(&ftm->delayedcounterwork,
				   msecs_to_jiffies(ftm->poll_interval));
}

/* Reports the hardware counter added the offset counter.
 *
 * The quadrature decodes does not use interrupts, because it cannot be
 * guaranteed that the counter won't flip between 0xFFFF and 0x0000 at a high
 * rate, causing Real Time performance degration. Instead the counter must be
 * read frequently enough - the assumption is 150 KHz input can be handled with
 * 100 ms read cycles.
 */
static void ftm_work_counter(struct ftm_quaddec *ftm,
			     struct counter_result *returndata)
{
	/* only 16bits filled in*/
	uint32_t hwcounter;
	uint8_t currentregion;

	mutex_lock(&ftm->ftm_quaddec_mutex);

	ftm_read(ftm, FTM_CNT, &hwcounter);

	/* Divide the counter in four regions:
	 *   0x0000-0x4000-0x8000-0xC000-0xFFFF
	 * When the hwcounter changes between region 0 and 3 there is an
	 * over/underflow
	 */
	currentregion = hwcounter / 0x4000;

	if (ftm->lastregion == 3 && currentregion == 0)
		ftm->counteroffset += 0x10000;

	if (ftm->lastregion == 0 && currentregion == 3)
		ftm->counteroffset -= 0x10000;

	ftm->lastregion = currentregion;

	if (returndata)
		returndata->value = ftm->counteroffset + hwcounter;

	ftm_work_reschedule(ftm);

	mutex_unlock(&ftm->ftm_quaddec_mutex);
}

/* wrapper around the real function */
static void ftm_work_counter_delay(struct work_struct *workptr)
{
	struct delayed_work *work;
	struct ftm_quaddec *ftm;

	work = container_of(workptr, struct delayed_work, work);
	ftm = container_of(work, struct ftm_quaddec, delayedcounterwork);

	ftm_work_counter(ftm, NULL);
}

/* must be called with mutex locked */
static void ftm_reset_counter(struct ftm_quaddec *ftm)
{
	ftm->counteroffset = 0;
	ftm->lastregion = 0;

	/* Reset hardware counter to CNTIN */
	ftm_write(ftm, FTM_CNT, 0x0);
}

static void ftm_quaddec_init(struct ftm_quaddec *ftm)
{
	ftm_clear_write_protection(ftm);

	/* Do not write in the region from the CNTIN register through the
	 * PWMLOAD register when FTMEN = 0.
	 */
	ftm_write(ftm, FTM_MODE, FTM_MODE_FTMEN); /* enable FTM */
	ftm_write(ftm, FTM_CNTIN, 0x0000);         /* zero init value */
	ftm_write(ftm, FTM_MOD, 0xffff);        /* max overflow value */
	ftm_write(ftm, FTM_CNT, 0x0);           /* reset counter value */
	ftm_write(ftm, FTM_SC, FTM_SC_PS_1);    /* prescale with x1 */
	/* Select quad mode */
	ftm_write(ftm, FTM_QDCTRL, FTM_QDCTRL_QUADEN);

	/* Unused features and reset to default section */
	ftm_write(ftm, FTM_POL, 0x0);     /* polarity is active high */
	ftm_write(ftm, FTM_FLTCTRL, 0x0); /* all faults disabled */
	ftm_write(ftm, FTM_SYNCONF, 0x0); /* disable all sync */
	ftm_write(ftm, FTM_SYNC, 0xffff);

	/* Lock the FTM */
	ftm_set_write_protection(ftm);
}

static int ftm_quaddec_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ftm_quaddec *ftm = iio_priv(indio_dev);
	struct counter_result counter;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		ftm_work_counter(ftm, &counter);
		if (mask == IIO_CHAN_INFO_RAW)
			counter.value &= 0xffff;

		*val = counter.value;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static uint32_t ftm_get_default_poll_interval(const struct ftm_quaddec *ftm)
{
	/* Read values from device tree */
	uint32_t val;
	const struct device_node *node = ftm->pdev->dev.of_node;

	if (of_property_read_u32(node, "poll-interval", &val))
		val = DEFAULT_POLL_INTERVAL;

	return val;
}

static ssize_t ftm_read_default_poll_interval(struct iio_dev *indio_dev,
					uintptr_t private,
					struct iio_chan_spec const *chan,
					char *buf)
{
	const struct ftm_quaddec *ftm = iio_priv(indio_dev);
	uint32_t val = ftm_get_default_poll_interval(ftm);

	return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t ftm_read_poll_interval(struct iio_dev *indio_dev,
				uintptr_t private,
				struct iio_chan_spec const *chan,
				char *buf)
{
	struct ftm_quaddec *ftm = iio_priv(indio_dev);

	uint32_t poll_interval = READ_ONCE(ftm->poll_interval);

	return snprintf(buf, PAGE_SIZE, "%u\n", poll_interval);
}

static ssize_t ftm_write_poll_interval(struct iio_dev *indio_dev,
				uintptr_t private,
				struct iio_chan_spec const *chan,
				const char *buf, size_t len)
{
	struct ftm_quaddec *ftm = iio_priv(indio_dev);
	uint32_t newpoll_interval;
	uint32_t default_interval;

	if (kstrtouint(buf, 10, &newpoll_interval) != 0) {
		dev_err(&ftm->pdev->dev, "poll_interval not a number: '%s'\n",
			buf);
		return -EINVAL;
	}

	/* Don't accept polling times below the default value to protect the
	 * system.
	 */
	default_interval = ftm_get_default_poll_interval(ftm);

	if (newpoll_interval < default_interval && newpoll_interval != 0)
		newpoll_interval = default_interval;

	mutex_lock(&ftm->ftm_quaddec_mutex);

	WRITE_ONCE(ftm->poll_interval, newpoll_interval);
	ftm_work_reschedule(ftm);

	mutex_unlock(&ftm->ftm_quaddec_mutex);

	return len;
}

static ssize_t ftm_write_reset(struct iio_dev *indio_dev,
				uintptr_t private,
				struct iio_chan_spec const *chan,
				const char *buf, size_t len)
{
	struct ftm_quaddec *ftm = iio_priv(indio_dev);

	/* Only "counter reset" is supported for now */
	if (!sysfs_streq(buf, "0")) {
		dev_warn(&ftm->pdev->dev, "Reset only accepts '0'\n");
		return -EINVAL;
	}

	mutex_lock(&ftm->ftm_quaddec_mutex);

	ftm_reset_counter(ftm);

	mutex_unlock(&ftm->ftm_quaddec_mutex);
	return len;
}

static int ftm_quaddec_get_prescaler(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ftm_quaddec *ftm = iio_priv(indio_dev);
	uint32_t scflags;

	ftm_read(ftm, FTM_SC, &scflags);

	return scflags & FTM_SC_PS_MASK;
}

static int ftm_quaddec_set_prescaler(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					unsigned int type)
{
	struct ftm_quaddec *ftm = iio_priv(indio_dev);

	uint32_t scflags;

	mutex_lock(&ftm->ftm_quaddec_mutex);

	ftm_read(ftm, FTM_SC, &scflags);

	scflags &= ~FTM_SC_PS_MASK;
	type &= FTM_SC_PS_MASK; /*just to be 100% sure*/

	scflags |= type;

	/* Write */
	ftm_clear_write_protection(ftm);
	ftm_write(ftm, FTM_SC, scflags);
	ftm_set_write_protection(ftm);

	/* Also resets the counter as it is undefined anyway now */
	ftm_reset_counter(ftm);

	mutex_unlock(&ftm->ftm_quaddec_mutex);
	return 0;
}

static const char * const ftm_quaddec_prescaler[] = {
	"1", "2", "4", "8", "16", "32", "64", "128"
};

static const struct iio_enum ftm_quaddec_prescaler_en = {
	.items = ftm_quaddec_prescaler,
	.num_items = ARRAY_SIZE(ftm_quaddec_prescaler),
	.get = ftm_quaddec_get_prescaler,
	.set = ftm_quaddec_set_prescaler,
};

static const struct iio_chan_spec_ext_info ftm_quaddec_ext_info[] = {
	{
		.name = "default_poll_interval",
		.shared = IIO_SHARED_BY_TYPE,
		.read = ftm_read_default_poll_interval,
	},
	{
		.name = "poll_interval",
		.shared = IIO_SHARED_BY_TYPE,
		.read = ftm_read_poll_interval,
		.write = ftm_write_poll_interval,
	},
	{
		.name = "reset",
		.shared = IIO_SEPARATE,
		.write = ftm_write_reset,
	},
	IIO_ENUM("prescaler", IIO_SEPARATE, &ftm_quaddec_prescaler_en),
	IIO_ENUM_AVAILABLE("prescaler", &ftm_quaddec_prescaler_en),
	{}
};

static const struct iio_chan_spec ftm_quaddec_channels = {
	.type = IIO_COUNT,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_PROCESSED),
	.ext_info = ftm_quaddec_ext_info,
	.indexed = 1,
};

static const struct iio_info ftm_quaddec_iio_info = {
	.read_raw = ftm_quaddec_read_raw,
};

static int ftm_quaddec_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ftm_quaddec *ftm;
	int ret;

	struct device_node *node = pdev->dev.of_node;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*ftm));
	if (!indio_dev)
		return -ENOMEM;

	ftm = iio_priv(indio_dev);

	platform_set_drvdata(pdev, ftm);

	ftm->pdev = pdev;
	ftm->big_endian = of_property_read_bool(node, "big-endian");
	ftm->counteroffset = 0;
	ftm->lastregion = 0;
	ftm->ftm_base = of_iomap(node, 0);
	if (!ftm->ftm_base)
		return -EINVAL;	

	ftm->poll_interval = ftm_get_default_poll_interval(ftm);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &ftm_quaddec_iio_info;
	indio_dev->num_channels = 1;
	indio_dev->channels = &ftm_quaddec_channels;
	
	ftm->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(ftm->ipg_clk)){
		ftm->ipg_clk = 0;
	}	
	
	if (ftm->ipg_clk)
		clk_prepare_enable(ftm->ipg_clk);
	
	ftm_read(ftm, FTM_SC, &ret);
	
	ftm_quaddec_init(ftm);
	
	mutex_init(&ftm->ftm_quaddec_mutex);
	INIT_DELAYED_WORK(&ftm->delayedcounterwork, ftm_work_counter_delay);

	ftm_work_reschedule(ftm);

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret) {
		cancel_delayed_work_sync(&ftm->delayedcounterwork);
		mutex_destroy(&ftm->ftm_quaddec_mutex);
		iounmap(ftm->ftm_base);
	}
	return ret;
}

static int ftm_quaddec_remove(struct platform_device *pdev)
{
	struct ftm_quaddec *ftm;
	struct iio_dev *indio_dev;

	ftm = (struct ftm_quaddec *)platform_get_drvdata(pdev);
	indio_dev = iio_priv_to_dev(ftm);
	/* Make sure no concurrent attribute reads happen*/
	devm_iio_device_unregister(&pdev->dev, indio_dev);

	cancel_delayed_work_sync(&ftm->delayedcounterwork);

	ftm_write(ftm, FTM_MODE, 0);

	mutex_destroy(&ftm->ftm_quaddec_mutex);
	iounmap(ftm->ftm_base);

	return 0;
}

static const struct of_device_id ftm_quaddec_match[] = {
	{ .compatible = "fsl,ftm-quaddec" },
	{},
};

static struct platform_driver ftm_quaddec_driver = {
	.driver = {
		.name = "ftm-quaddec",
		.owner = THIS_MODULE,
		.of_match_table = ftm_quaddec_match,
	},
	.probe = ftm_quaddec_probe,
	.remove = ftm_quaddec_remove,
};

module_platform_driver(ftm_quaddec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kjeld Flarup <kfa@deif.com");
MODULE_AUTHOR("Patrick Havelange <patrick.havelange@essensium.com");

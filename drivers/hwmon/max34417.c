// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hardware monitoring driver for the Analog Devices (Maxim) MAX34417
 * four-channel power accumulator.
 *
 * Based on the max34417 driver in the Microsoft Surface Duo kernel,
 * Copyright (c) 2020 Microsoft Corporation.
 *
 * The device multiplies current-sense and voltage samples (1024 per second)
 * and accumulates the products. An UPDATE command latches the accumulators
 * and the sample count; the driver does that once per update interval and
 * reports the average power and the last voltage of each channel.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define MAX34417_CMD_UPDATE		0x00
#define MAX34417_CMD_CONTROL		0x01
#define MAX34417_CMD_ACC_COUNT		0x02
#define MAX34417_CMD_DID		0x0f
#define MAX34417_CMD_BULK_POWER		0x10
#define MAX34417_CMD_BULK_VOLTAGE	0x11

#define MAX34417_CTRL_MODE		BIT(7)	/* MAX34417 (not MAX34407) mode */

#define MAX34417_DID_ID(did)		(((did) >> 3) & 0x1f)
#define MAX34417_DEVICE_ID		0x07

#define MAX34417_CHANNELS		4
#define MAX34417_VOLTAGE_BITS		14
#define MAX34417_VOLTAGE_FULL_SCALE_MV	24000
/* Average power full scale is 2400 W with a 1 milliohm sense resistor */
#define MAX34417_POWER_FULL_SCALE	2400000000000ULL	/* uW * uOhm */
#define MAX34417_POWER_BITS		30

#define MAX34417_UPDATE_DELAY_MS	3
#define MAX34417_DEFAULT_INTERVAL_MS	1000
#define MAX34417_MIN_INTERVAL_MS	10

static unsigned int default_shunt_uohm = 1000;
module_param(default_shunt_uohm, uint, 0444);
MODULE_PARM_DESC(default_shunt_uohm,
		 "Sense resistor for channels without firmware description, in micro-ohms (default 1000)");

struct max34417_data {
	struct i2c_client *client;
	struct delayed_work work;
	struct mutex lock;		/* protects the readings and the interval */
	unsigned long interval_ms;
	u32 shunt_uohm[MAX34417_CHANNELS];
	const char *label[MAX34417_CHANNELS];
	bool enabled[MAX34417_CHANNELS];
	long power_uw[MAX34417_CHANNELS];
	long voltage_mv[MAX34417_CHANNELS];
	bool valid;
};

static int max34417_update(struct max34417_data *data)
{
	struct i2c_client *client = data->client;
	u8 count_buf[4], power_buf[29], voltage_buf[9];
	u32 count;
	int i, ret;

	ret = i2c_smbus_write_byte(client, MAX34417_CMD_UPDATE);
	if (ret)
		return ret;

	msleep(MAX34417_UPDATE_DELAY_MS);

	/* The block reads start with a byte the device fills with the length */
	ret = i2c_smbus_read_i2c_block_data(client, MAX34417_CMD_ACC_COUNT,
					    sizeof(count_buf), count_buf);
	if (ret < 0)
		return ret;
	ret = i2c_smbus_read_i2c_block_data(client, MAX34417_CMD_BULK_POWER,
					    sizeof(power_buf), power_buf);
	if (ret < 0)
		return ret;
	ret = i2c_smbus_read_i2c_block_data(client, MAX34417_CMD_BULK_VOLTAGE,
					    sizeof(voltage_buf), voltage_buf);
	if (ret < 0)
		return ret;

	count = get_unaligned_be24(&count_buf[1]);

	mutex_lock(&data->lock);
	for (i = 0; i < MAX34417_CHANNELS; i++) {
		/* 56-bit accumulator, 7 bytes per channel */
		u64 acc = get_unaligned_be64(&power_buf[1 + 7 * i]) >> 8;
		u16 raw = get_unaligned_be16(&voltage_buf[1 + 2 * i]) >> 2;
		u64 scale = div_u64(MAX34417_POWER_FULL_SCALE, data->shunt_uohm[i]);

		data->power_uw[i] = count ?
			(div_u64(acc, count) * scale) >> MAX34417_POWER_BITS : 0;
		data->voltage_mv[i] = (raw * MAX34417_VOLTAGE_FULL_SCALE_MV) >>
				      MAX34417_VOLTAGE_BITS;
	}
	data->valid = true;
	mutex_unlock(&data->lock);

	return 0;
}

static void max34417_work(struct work_struct *work)
{
	struct max34417_data *data = container_of(to_delayed_work(work),
						  struct max34417_data, work);
	unsigned long interval;
	int ret;

	ret = max34417_update(data);
	if (ret)
		dev_warn_ratelimited(&data->client->dev, "update failed: %d\n", ret);

	mutex_lock(&data->lock);
	interval = data->interval_ms;
	mutex_unlock(&data->lock);

	queue_delayed_work(system_freezable_wq, &data->work, msecs_to_jiffies(interval));
}

static int max34417_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&data->lock);
	switch (type) {
	case hwmon_chip:
		if (attr == hwmon_chip_update_interval)
			*val = data->interval_ms;
		else
			ret = -EOPNOTSUPP;
		break;
	case hwmon_power:
		if (!data->valid)
			ret = -EAGAIN;
		else if (attr == hwmon_power_average)
			*val = data->power_uw[channel];
		else
			ret = -EOPNOTSUPP;
		break;
	case hwmon_in:
		if (!data->valid)
			ret = -EAGAIN;
		else if (attr == hwmon_in_input)
			*val = data->voltage_mv[channel];
		else
			ret = -EOPNOTSUPP;
		break;
	default:
		ret = -EOPNOTSUPP;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int max34417_read_string(struct device *dev, enum hwmon_sensor_types type,
				u32 attr, int channel, const char **str)
{
	struct max34417_data *data = dev_get_drvdata(dev);

	if (!data->label[channel])
		return -EOPNOTSUPP;

	*str = data->label[channel];

	return 0;
}

static int max34417_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct max34417_data *data = dev_get_drvdata(dev);

	if (type != hwmon_chip || attr != hwmon_chip_update_interval)
		return -EOPNOTSUPP;

	mutex_lock(&data->lock);
	data->interval_ms = clamp_val(val, MAX34417_MIN_INTERVAL_MS, 60 * MSEC_PER_SEC);
	mutex_unlock(&data->lock);

	mod_delayed_work(system_freezable_wq, &data->work, 0);

	return 0;
}

static umode_t max34417_is_visible(const void *drvdata, enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	const struct max34417_data *data = drvdata;

	switch (type) {
	case hwmon_chip:
		if (attr == hwmon_chip_update_interval)
			return 0644;
		break;
	case hwmon_power:
		if (!data->enabled[channel])
			return 0;
		if (attr == hwmon_power_average)
			return 0444;
		if (attr == hwmon_power_label && data->label[channel])
			return 0444;
		break;
	case hwmon_in:
		if (!data->enabled[channel])
			return 0;
		if (attr == hwmon_in_input)
			return 0444;
		if (attr == hwmon_in_label && data->label[channel])
			return 0444;
		break;
	default:
		break;
	}

	return 0;
}

static const struct hwmon_channel_info * const max34417_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_UPDATE_INTERVAL),
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_AVERAGE | HWMON_P_LABEL,
			   HWMON_P_AVERAGE | HWMON_P_LABEL,
			   HWMON_P_AVERAGE | HWMON_P_LABEL,
			   HWMON_P_AVERAGE | HWMON_P_LABEL),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	NULL
};

static const struct hwmon_ops max34417_hwmon_ops = {
	.is_visible = max34417_is_visible,
	.read = max34417_read,
	.read_string = max34417_read_string,
	.write = max34417_write,
};

static const struct hwmon_chip_info max34417_chip_info = {
	.ops = &max34417_hwmon_ops,
	.info = max34417_info,
};

/*
 * Channels are described by child nodes with "reg" (1-4), and optionally
 * "label" and "shunt-resistor-micro-ohms". Without any, all four channels
 * are enabled with the default_shunt_uohm module parameter.
 */
static int max34417_parse_channels(struct device *dev, struct max34417_data *data)
{
	u32 reg, shunt;
	int i;

	device_for_each_child_node_scoped(dev, child) {
		if (fwnode_property_read_u32(child, "reg", &reg) || reg < 1 ||
		    reg > MAX34417_CHANNELS)
			return dev_err_probe(dev, -EINVAL, "invalid channel reg\n");

		i = reg - 1;
		data->enabled[i] = true;
		data->shunt_uohm[i] = default_shunt_uohm;
		if (!fwnode_property_read_u32(child, "shunt-resistor-micro-ohms", &shunt)) {
			if (!shunt)
				return dev_err_probe(dev, -EINVAL, "channel %u: shunt is 0\n", reg);
			data->shunt_uohm[i] = shunt;
		}
		fwnode_property_read_string(child, "label", &data->label[i]);
	}

	if (!device_get_child_node_count(dev)) {
		for (i = 0; i < MAX34417_CHANNELS; i++) {
			data->enabled[i] = true;
			data->shunt_uohm[i] = default_shunt_uohm;
		}
	}

	for (i = 0; i < MAX34417_CHANNELS; i++)
		if (!data->shunt_uohm[i])
			data->shunt_uohm[i] = default_shunt_uohm ?: 1000;

	return 0;
}

static void max34417_cancel_work(void *work)
{
	cancel_delayed_work_sync(work);
}

static int max34417_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max34417_data *data;
	struct device *hwmon_dev;
	int ret, did, ctrl;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE |
						      I2C_FUNC_SMBUS_BYTE_DATA |
						      I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -ENODEV;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	data->interval_ms = MAX34417_DEFAULT_INTERVAL_MS;
	ret = devm_mutex_init(dev, &data->lock);
	if (ret)
		return ret;

	ret = max34417_parse_channels(dev, data);
	if (ret)
		return ret;

	did = i2c_smbus_read_byte_data(client, MAX34417_CMD_DID);
	if (did < 0)
		return dev_err_probe(dev, did, "cannot read device id\n");
	if (MAX34417_DID_ID(did) != MAX34417_DEVICE_ID)
		return dev_err_probe(dev, -ENODEV, "unexpected device id 0x%02x\n", did);

	ctrl = i2c_smbus_read_byte_data(client, MAX34417_CMD_CONTROL);
	if (ctrl < 0)
		return dev_err_probe(dev, ctrl, "cannot read control register\n");
	if (!(ctrl & MAX34417_CTRL_MODE)) {
		ret = i2c_smbus_write_byte_data(client, MAX34417_CMD_CONTROL,
						ctrl | MAX34417_CTRL_MODE);
		if (ret)
			return dev_err_probe(dev, ret, "cannot write control register\n");
	}

	INIT_DELAYED_WORK(&data->work, max34417_work);
	ret = devm_add_action_or_reset(dev, max34417_cancel_work, &data->work);
	if (ret)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name, data,
							 &max34417_chip_info, NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	/* Restart accumulation now so the first reading covers one interval */
	i2c_smbus_write_byte(client, MAX34417_CMD_UPDATE);
	queue_delayed_work(system_freezable_wq, &data->work,
			   msecs_to_jiffies(data->interval_ms));

	dev_info(dev, "MAX34417 revision %d\n", did & 0x7);

	return 0;
}

static int max34417_suspend(struct device *dev)
{
	struct max34417_data *data = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&data->work);

	return 0;
}

static int max34417_resume(struct device *dev)
{
	struct max34417_data *data = dev_get_drvdata(dev);

	/* Restart accumulation so the next reading does not average over the sleep */
	i2c_smbus_write_byte(data->client, MAX34417_CMD_UPDATE);
	queue_delayed_work(system_freezable_wq, &data->work,
			   msecs_to_jiffies(data->interval_ms));

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(max34417_pm_ops, max34417_suspend, max34417_resume);

static const struct i2c_device_id max34417_id[] = {
	{ "max34417" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max34417_id);

static const struct of_device_id max34417_of_match[] = {
	{ .compatible = "adi,max34417" },
	{ }
};
MODULE_DEVICE_TABLE(of, max34417_of_match);

static struct i2c_driver max34417_driver = {
	.driver = {
		.name = "max34417",
		.of_match_table = max34417_of_match,
		.pm = pm_sleep_ptr(&max34417_pm_ops),
	},
	.probe = max34417_probe,
	.id_table = max34417_id,
};
module_i2c_driver(max34417_driver);

MODULE_DESCRIPTION("MAX34417 four-channel power accumulator driver");
MODULE_LICENSE("GPL");

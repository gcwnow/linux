/*
 * it6610-i2c.c - Driver for using ITE Tech's IT6610 HDMI transmitter via I2C
 *
 * Copyright (c) 2013 Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <video/panel-it6610.h>


static int it6610_read_ids(struct i2c_client *client)
{
	__u16 vendorID, deviceID;
	__u8 revisionID;

	__u8 reg_buf = 0;
	__u8 val_buf[4];
	struct i2c_msg msgs[] = {
		{ /* write register number */
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(reg_buf),
			.buf = &reg_buf,
		},
		{ /* read register contents */
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(val_buf),
			.buf = val_buf,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) return ret;
	if (ret < ARRAY_SIZE(msgs)) return -EIO;

	vendorID = val_buf[0] | (val_buf[1] << 8);
	deviceID = val_buf[2] | ((val_buf[3] & 0x0F) << 8);
	revisionID = val_buf[3] >> 4;

	dev_info(&client->dev, "Vendor 0x%04X, device 0x%03X, revision 0x%1X\n",
		 vendorID, deviceID, revisionID);

	if (vendorID != 0xCA00)
		return 1;
	if (deviceID != 0x0611)
		return 2;

	return 0;
}

// TODO: Support writing to regs >= 0x100.
static int it6610_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	__u8 buf[] = { reg, val };
	struct i2c_msg msgs[] = {
		{ /* write register number and contents */
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(buf),
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) return ret;
	if (ret < ARRAY_SIZE(msgs)) return -EIO;

	return 0;
}

static int it6610_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct it6610_i2c_platform_data *pdata = dev->platform_data;
	int gpio_reset;
	int ret;

	if (pdata) {
		gpio_reset = pdata->gpio_reset;
	} else {
		dev_info(dev, "No platform data\n");
		gpio_reset = -1;
	}

	if (gpio_is_valid(gpio_reset)) {
		ret = devm_gpio_request(dev, gpio_reset, "IT6610 reset");
		if (ret) {
			dev_err(dev,
				"Failed to request IT6610 reset pin %d: %d\n",
				gpio_reset, ret);
			gpio_reset = -1;
		}
	}

	if (gpio_is_valid(gpio_reset)) {
		/* Perform a hardware reset. */
		gpio_direction_output(gpio_reset, 0);
		msleep(30);
		gpio_set_value(gpio_reset, 1);
		msleep(10);
	}

	ret = it6610_read_ids(client);
	if (ret < 0) {
		dev_err(dev, "Error fetching IDs: %d\n", ret);
		return ret;
	} else if (ret > 0) {
		dev_err(dev, "Device is not IT6610\n");
		return -ENODEV;
	}

	/* Perform a software reset. */
	ret = it6610_i2c_write(client, 0x04, 0x3C);
	if (ret < 0) {
		dev_err(dev, "Error performing soft reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int it6610_i2c_remove(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int it6610_suspend(struct device *dev)
{
	return 0;
}

static int it6610_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(it6610_pm_ops, it6610_suspend, it6610_resume);
#define IT6610_PM_OPS (&it6610_pm_ops)

#else

#define IT6610_PM_OPS NULL

#endif

static const struct i2c_device_id it6610_id[] = {
	{ "it6610-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, it6610_id);

static struct i2c_driver it6610_i2c_driver = {
	.probe = it6610_i2c_probe,
	.remove = it6610_i2c_remove,
	.id_table = it6610_id,
	.driver = {
		.name	= "it6610-i2c",
		.owner	= THIS_MODULE,
		.pm	= IT6610_PM_OPS,
	},
};

module_i2c_driver(it6610_i2c_driver);

MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("IT6610 HDMI transmitter driver");
MODULE_LICENSE("GPL");

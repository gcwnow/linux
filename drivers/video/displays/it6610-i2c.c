/*
 * it6610-i2c.c - Driver for using ITE Tech's IT6610 HDMI transmitter via I2C
 *
 * Copyright (c) 2013 Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/i2c.h>
#include <linux/module.h>


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
	int err;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err < 0) return err;
	if (err < ARRAY_SIZE(msgs)) return -EIO;

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

static int it6610_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int err;

	err = it6610_read_ids(client);
	if (err < 0) {
		dev_err(&client->dev, "Error fetching IDs: %d\n", err);
		return err;
	} else if (err > 0) {
		dev_err(&client->dev, "Device is not IT6610\n");
		return -ENODEV;
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

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
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <video/panel-it6610.h>


enum it6610_state {
	UNKNOWN = 0,
	NO_RECEIVER,
	RECEIVER_READY,
	ACTIVE,
};

struct it6610_i2c {
	struct i2c_client *client;
	struct regmap *regmap;
	enum it6610_state state;
	struct delayed_work check_state_work;
};

static int it6610_read_ids(struct device *dev, struct regmap *regmap)
{
	__u16 vendorID, deviceID;
	__u8 revisionID;

	__u8 val_buf[4];
	int ret;

	ret = regmap_raw_read(regmap, 0x000, val_buf, sizeof(val_buf));
	if (ret < 0) return ret;

	vendorID = val_buf[0] | (val_buf[1] << 8);
	deviceID = val_buf[2] | ((val_buf[3] & 0x0F) << 8);
	revisionID = val_buf[3] >> 4;

	dev_info(dev, "Vendor 0x%04X, device 0x%03X, revision 0x%1X\n",
		 vendorID, deviceID, revisionID);

	if (vendorID != 0xCA00)
		return 1;
	if (deviceID != 0x0611)
		return 2;

	return 0;
}

static void it6610_i2c_check_state_work(struct work_struct *work)
{
	struct it6610_i2c *it6610 = container_of(work, struct it6610_i2c,
						 check_state_work.work);
	struct regmap *regmap = it6610->regmap;
	unsigned int status;
	enum it6610_state newstate;
	int err;

	if ((err = regmap_read(regmap, 0x00E, &status)))
		goto exit_err;

	switch (status & 0x70) {
	case 0x70: /* video input is stable */
		newstate = ACTIVE;
		break;
	case 0x60: /* receiver is plugged and sensed */
		newstate = RECEIVER_READY;
		break;
	default:
		newstate = NO_RECEIVER;
		break;
	}
	dev_info(&it6610->client->dev,
		"System status %02X; old state %d; new state %d\n",
		status, it6610->state, newstate);

	if (newstate == it6610->state)
		return;

	if (newstate == NO_RECEIVER) {
		/* Disable video clock. */
		if ((err = regmap_write(regmap, 0x004, 0x1D)))
			goto exit_err;
	} else if (it6610->state < RECEIVER_READY) {
		/* Enable video clock. */
		if ((err = regmap_write(regmap, 0x004, 0x15)))
			goto exit_err;
	}

	if (newstate == ACTIVE) {
		/* Enable analog front end. */
		if ((err = regmap_write(regmap, 0x061, 0x00)))
			goto exit_err;
		/* Disable AV mute. */
		if ((err = regmap_write(regmap, 0x0C1, 0x00)))
			goto exit_err;
	} else if (it6610->state == ACTIVE || it6610->state == UNKNOWN) {
		/* Enable AV mute. */
		if ((err = regmap_write(regmap, 0x0C1, 0x01)))
			goto exit_err;
		/* Disable analog front end. */
		if ((err = regmap_write(regmap, 0x061, 0x10)))
			goto exit_err;
	}

	it6610->state = newstate;

	return;

exit_err:
	dev_warn(&it6610->client->dev,
		 "Error %d accessing registers in state %d\n",
		 err, it6610->state);

	it6610->state = UNKNOWN;

	/* Try again later. */
	schedule_delayed_work(&it6610->check_state_work, HZ);
}

static irqreturn_t it6610_i2c_irq(int irq, void *data)
{
	struct it6610_i2c *it6610 = data;
	struct regmap *regmap = it6610->regmap;
	__u8 status[3];
	__u8 clear[3] = { 0, 0, 0x0D };
	int err;

	/* Read status of all interrupts. */
	err = regmap_raw_read(regmap, 0x006, status, sizeof(status));
	if (err < 0) {
		dev_err(&it6610->client->dev,
			"Error reading interrupt status: %d\n", err);
		goto err_disable_irqs;
	}

	/* Handle active interrupts. */
	if (status[0] & 0x01) {
		dev_info(&it6610->client->dev, "HPD interrupt\n");
		clear[0] |= 0x01;
	}
	if (status[0] & 0x02) {
		dev_info(&it6610->client->dev, "RxSEN interrupt\n");
		clear[0] |= 0x02;
	}
	if (status[2] & 0x10) {
		dev_info(&it6610->client->dev, "VidStable interrupt\n");
		clear[1] |= 0x40;
	}

	/*
	 * Wake up worker.
	 * Include a delay, since VidStable tends to flip back to 0 for a
	 * moment after first becoming 1.
	 */
	cancel_delayed_work(&it6610->check_state_work);
	schedule_delayed_work(&it6610->check_state_work, msecs_to_jiffies(100));

	/* Clear interrupts. */
	if (clear[0] || clear[1]) {
		err = regmap_raw_write(regmap, 0x00C, clear, sizeof(clear));
		if (err < 0) {
			dev_err(&it6610->client->dev,
				"Error clearing interrupt: %d\n", err);
			goto err_disable_irqs;
		}

		return IRQ_HANDLED;
	}

err_disable_irqs:
	/*
	 * If we cannot acknowledge the interrupt, do damage control by
	 * disabling the IRQ.
	 */
	dev_err(&it6610->client->dev, "Disabling IRQ\n");
	disable_irq(it6610->client->irq);
	return IRQ_NONE;
}

// TODO: Support writing to regs >= 0x100.
static const struct regmap_config it6610_i2c_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
};

static int it6610_i2c_set_timing(struct regmap *regmap)
{
	/* Hardcoded timings for 640x480p. */
	static const unsigned int hds = 144;
	static const unsigned int hde = hds + 640;
	static const unsigned int vds = 35;
	static const unsigned int vde = vds + 480;

	/* Configure TMDS for clock rates below 80 MHz. */
	static const __u8 tmds_val[4] = { 0x19, 0x03, 0x1E, 0x00 };

	/*
	 * Regenerate DE from hsync and vsync.
	 * Although the DE signal we pass is usable, we have one extra display
	 * line in our setup as a trick to get interrupts at the right moment.
	 * That line should be omitted from the HDMI output to avoid problems
	 * with timings.
	 */
	static const __u8 timing_val[0xA0 - 0x90] = {
		0x01,
		0,
		(hds - 2) & 0x0FF,
		(hde - 2) & 0x0FF,
		((hde - 2) & 0xF00) >> 4 | ((hds - 2) & 0xF00) >> 8,
		0,
		0,
		0,
		0,
		0,
		(vds - 1) & 0x0FF,
		(vde - 1) & 0x0FF,
		((vde - 1) & 0xF00) >> 4 | ((vds - 1) & 0xF00) >> 8,
		0xFF,
		0xFF,
		0xFF,
	};

	int ret;

	ret = regmap_raw_write(regmap, 0x62, tmds_val, sizeof(tmds_val));
	if (ret < 0)
		return ret;

	ret = regmap_raw_write(regmap, 0x90, timing_val, sizeof(timing_val));
	if (ret < 0)
		return ret;

	return 0;
}

static int it6610_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct it6610_i2c_platform_data *pdata = dev->platform_data;
	struct it6610_i2c *it6610;
	struct regmap *regmap;
	int gpio_reset;
	int ret;

	if (pdata) {
		gpio_reset = pdata->gpio_reset;
	} else {
		dev_info(dev, "No platform data\n");
		gpio_reset = -1;
	}

	it6610 = devm_kzalloc(dev, sizeof(*it6610), GFP_KERNEL);
	if (!it6610)
		return -ENOMEM;
	it6610->client = client;
	INIT_DELAYED_WORK(&it6610->check_state_work,
			  it6610_i2c_check_state_work);

	regmap = devm_regmap_init_i2c(client, &it6610_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}
	it6610->regmap = regmap;

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

	ret = it6610_read_ids(dev, regmap);
	if (ret < 0) {
		dev_err(dev, "Error fetching IDs: %d\n", ret);
		return ret;
	} else if (ret > 0) {
		dev_err(dev, "Device is not IT6610\n");
		return -ENODEV;
	}

	/* Perform a software reset. */
	if ((ret = regmap_write(regmap, 0x004, 0x3C)) < 0) {
		dev_err(dev, "Error performing soft reset: %d\n", ret);
		return ret;
	}

	if ((ret = it6610_i2c_set_timing(regmap)) < 0) goto err_init;

	/* Unmask interrupts: hot plug, receiver sense, video stable. */
	if ((ret = regmap_write(regmap, 0x009, 0xFC)) < 0) goto err_init;
	if ((ret = regmap_write(regmap, 0x00B, 0xF7)) < 0) goto err_init;

	/*
	 * Request a threaded IRQ, since our register access is slow and
	 * the events we respond to are not urgent.
	 */
	ret = devm_request_threaded_irq(dev, client->irq, NULL, it6610_i2c_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"it6610-irq", it6610);
	if (ret) {
		dev_err(dev, "Failed to request IRQ %d: %d\n",
			client->irq, ret);
		return ret;
	}

	/* Determine initial state. */
	schedule_delayed_work(&it6610->check_state_work, 0);

	return 0;

err_init:
	dev_err(dev, "Error writing registers during init: %d\n", ret);
	return ret;
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

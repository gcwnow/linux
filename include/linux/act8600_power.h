/*
 * drivers/power/act8600_power.h -- Core interface for ACT8600
 *
 * Copyright 2010 Ingenic Semiconductor LTD.
 * Copyright 2012 Maarten ter Huurne <maarten@treewalker.org>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __ACT8600_POWER_H__
#define __ACT8600_POWER_H__

#include <linux/types.h>

#define ACT8600_NAME		"act8600"
#define ACT8600_I2C_ADDR	0x5A

struct act8600_outputs_t {
	int outnum;
	int value;
	bool enable;
};

struct act8600_platform_pdata_t {
	struct act8600_outputs_t *outputs;
	int nr_outputs;
};

/**
 * act8600_output_enable - Enable or disable one of the power outputs.
 * @outnum: output pin: 1-8
 * @enable: true to enable, false to disable
 * @returns zero on success and error code upon failure
 */
int act8600_output_enable(int outnum, bool enable);

#endif  /* __ACT8600_POWER_H__ */

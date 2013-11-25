/*
 * linkdev platform data.
 *
 * Copyright (C) 2013, Paul Cercueil <paul@crapouillou.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_JZ4770_LINKDEV_H__
#define __ASM_MACH_JZ4770_LINKDEV_H__

struct linkdev_pdata_device_info {
	const char *name; /* Name of the input device, e.g. gpio-keys */
	const char **handlers_blacklist;
	unsigned int nb_handlers;
};

struct linkdev_pdata_abs_map {
	const struct linkdev_pdata_device_info *device; /* input device of origin */
	unsigned int axis; /* axis of the input device to map to */
};

struct linkdev_platform_data {
	const struct linkdev_pdata_device_info *devices;
	unsigned int nb_devices;

	/* Event translation map for keys.
	 * key_map[0] events will be transformed into key_map[1].
	 */
	const signed short (*key_map)[2];
	unsigned int key_map_size;

	/* Event translation map for joystick axis */
	const struct linkdev_pdata_abs_map *abs_map;
	unsigned int abs_map_size;

	void *private;
};

#endif /* __ASM_MACH_JZ4770_LINKDEV_H__ */

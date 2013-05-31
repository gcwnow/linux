/*
 * Register cache access API - flat caching support
 *
 * Copyright 2012 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/seq_file.h>

#include "internal.h"

static int regcache_flat_init(struct regmap *map)
{
	int i;
	unsigned int *cache;

	map->cache = kzalloc(sizeof(unsigned int) * (map->max_register + 1),
			     GFP_KERNEL);
	if (!map->cache)
		return -ENOMEM;

	cache = map->cache;

	for (i = 0; i < map->num_reg_defaults; i++)
		cache[map->reg_defaults[i].reg] = map->reg_defaults[i].def;

	return 0;
}

static int regcache_flat_exit(struct regmap *map)
{
	kfree(map->cache);
	map->cache = NULL;

	return 0;
}

static int regcache_flat_read(struct regmap *map,
			      unsigned int reg, unsigned int *value)
{
	unsigned int *cache = map->cache;

	*value = cache[reg];

	return 0;
}

static int regcache_flat_write(struct regmap *map, unsigned int reg,
			       unsigned int value)
{
	unsigned int *cache = map->cache;

	cache[reg] = value;

	return 0;
}

static int regcache_flat_sync(struct regmap *map, unsigned int min,
			      unsigned int max)
{
	unsigned int *cache = map->cache;
	unsigned int reg;

	for (reg = min; reg <= max; reg++) {
		unsigned int val;
		int ret;

		if (regmap_volatile(map, reg))
			continue;

		val = cache[reg];

		/* Is this the hardware default?  If so skip. */
		ret = regcache_lookup_reg(map, reg);
		if (ret >= 0 && val == map->reg_defaults[ret].def)
			continue;

		map->cache_bypass = 1;
		ret = _regmap_write(map, reg, val);
		map->cache_bypass = 0;
		if (ret)
			return ret;
		dev_dbg(map->dev, "Synced register %#x, value %#x\n", reg, val);
	}

	return 0;
}

struct regcache_ops regcache_flat_ops = {
	.type = REGCACHE_FLAT,
	.name = "flat",
	.init = regcache_flat_init,
	.exit = regcache_flat_exit,
	.read = regcache_flat_read,
	.write = regcache_flat_write,
	.sync = regcache_flat_sync,
};

#ifndef __LINUX_CLK_JZ4740_TCU_H_
#define __LINUX_CLK_JZ4740_TCU_H_

#include <linux/bitops.h>
#include <linux/regmap.h>

#define TCU_TER_OFFSET		0x0
#define TCU_TESR_OFFSET		0x4
#define TCU_TECR_OFFSET		0x8

static inline int tcu_timer_enable(struct regmap *ter, unsigned int channel)
{
	return regmap_update_bits(ter, TCU_TESR_OFFSET,
			BIT(channel), BIT(channel));
}

static inline int tcu_timer_disable(struct regmap *ter, unsigned int channel)
{
	return regmap_update_bits(ter, TCU_TECR_OFFSET,
			BIT(channel), BIT(channel));
}

static inline int tcu_timer_enabled(struct regmap *ter, unsigned int channel)
{
	unsigned int val;
	int err;

	err = regmap_read(ter, TCU_TER_OFFSET, &val);
	if (err)
		return err;
	else
		return !!(val & BIT(channel));

}

#endif /* __LINUX_CLK_JZ4740_TCU_H_ */

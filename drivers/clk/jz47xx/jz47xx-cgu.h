/*
 * Ingenic jz47xx series CGU driver
 *
 * Copyright (c) 2013-2015 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __DRIVERS_CLK_JZ47XX_JZ47XX_CGU_H__
#define __DRIVERS_CLK_JZ47XX_JZ47XX_CGU_H__

#include <linux/of.h>
#include <linux/spinlock.h>

/**
 * struct jz47xx_cgu_pll_info - information about a PLL
 * @reg: the offset of the PLL's control register within the CGU
 * @m_shift: the number of bits to shift the multiplier value by (ie. the
 *           index of the lowest bit of the multiplier value in the PLL's
 *           control register)
 * @m_bits: the size of the multiplier field in bits
 * @m_offset: the multiplier value which encodes to 0 in the PLL's control
 *            register
 * @n_shift: the number of bits to shift the divider value by (ie. the
 *           index of the lowest bit of the divider value in the PLL's
 *           control register)
 * @n_bits: the size of the divider field in bits
 * @n_offset: the divider value which encodes to 0 in the PLL's control
 *            register
 * @od_shift: the number of bits to shift the post-VCO divider value by (ie.
 *            the index of the lowest bit of the post-VCO divider value in
 *            the PLL's control register)
 * @od_bits: the size of the post-VCO divider field in bits
 * @od_max: the maximum post-VCO divider value
 * @od_encoding: a pointer to an array mapping post-VCO divider values to
 *               their encoded values in the PLL control register, or -1 for
 *               unsupported values
 * @bypass_bit: the index of the bypass bit in the PLL control register
 * @enable_bit: the index of the enable bit in the PLL control register
 * @stable_bit: the index of the stable bit in the PLL control register
 */
struct jz47xx_cgu_pll_info {
	unsigned reg;
	unsigned m_shift, m_bits, m_offset;
	unsigned n_shift, n_bits, n_offset;
	unsigned od_shift, od_bits, od_max;
	const s8 *od_encoding;
	unsigned bypass_bit;
	unsigned enable_bit;
	unsigned stable_bit;
};

/**
 * struct jz47xx_cgu_mux_info - information about a clock mux
 * @reg: offset of the mux control register within the CGU
 * @shift: number of bits to shift the mux value by (ie. the index of
 *         the lowest bit of the mux value within its control register)
 * @bits: the size of the mux value in bits
 */
struct jz47xx_cgu_mux_info {
	unsigned reg;
	unsigned shift:5;
	unsigned bits:5;
};

/**
 * struct jz47xx_cgu_div_info - information about a divider
 * @reg: offset of the divider control register within the CGU
 * @shift: number of bits to shift the divide value by (ie. the index of
 *         the lowest bit of the divide value within its control register)
 * @bits: the size of the divide value in bits
 * @ce_bit: the index of the change enable bit within reg, or -1 is there
 *          isn't one
 * @busy_bit: the index of the busy bit within reg, or -1 is there isn't one
 * @stop_bit: the index of the stop bit within reg, or -1 is there isn't one
 */
struct jz47xx_cgu_div_info {
	unsigned reg;
	unsigned shift:5;
	unsigned bits:5;
	int ce_bit:6;
	int busy_bit:6;
	int stop_bit:6;
};

/**
 * struct jz47xx_cgu_gate_info - information about a clock gate
 * @reg: offset of the gate control register within the CGU
 * @bit: offset of the bit in the register that controls the gate
 */
struct jz47xx_cgu_gate_info {
	unsigned reg;
	unsigned bit:5;
};

/**
 * struct jz47xx_cgu_custom_info - information about a custom (SoC) clock
 */
struct jz47xx_cgu_custom_info {
	struct clk_ops *clk_ops;
};

/**
 * struct jz47xx_cgu_clk_info - information about a clock
 * @name: name of the clock
 * @type: a bitmask formed from CGU_CLK_* values
 * @parents: an array of the indices of potential parents of this clock
 *           within the clock_info array of the CGU, or -1 in entries
 *           which correspond to no valid parent
 * @pll: information valid if type includes CGU_CLK_PLL
 * @gate: information valid if type includes CGU_CLK_GATE
 * @mux: information valid if type includes CGU_CLK_MUX
 * @div: information valid if type includes CGU_CLK_DIV
 */
struct jz47xx_cgu_clk_info {
	const char *name;

	enum {
		CGU_CLK_NONE		= 0,
		CGU_CLK_EXT		= (1 << 0),
		CGU_CLK_PLL		= (1 << 1),
		CGU_CLK_GATE		= (1 << 2),
		CGU_CLK_MUX		= (1 << 3),
		CGU_CLK_MUX_GLITCHFREE	= (1 << 4),
		CGU_CLK_DIV		= (1 << 5),
		CGU_CLK_CUSTOM		= (1 << 6),
	} type;

	int parents[4];

	union {
		struct jz47xx_cgu_pll_info pll;

		struct {
			struct jz47xx_cgu_gate_info gate;
			struct jz47xx_cgu_mux_info mux;
			struct jz47xx_cgu_div_info div;
		};

		struct jz47xx_cgu_custom_info custom;
	};
};

/**
 * struct jz47xx_cgu - data about the CGU
 * @np: the device tree node that caused the CGU to be probed
 * @base: the ioremap'ed base address of the CGU registers
 * @clock_info: an array containing information about implemented clocks
 * @clocks: used to provide clocks to DT, allows lookup of struct clk*
 * @gate_lock: lock to be held whilst (un)gating a clock
 * @divmux_lock: lock to be held whilst re-muxing of rate-changing a clock
 */
struct jz47xx_cgu {
	struct device_node *np;
	void __iomem *base;

	const struct jz47xx_cgu_clk_info *clock_info;
	struct clk_onecell_data clocks;

	spinlock_t divmux_lock;		/* must be held when changing a divide
					   or re-muxing a clock */
	spinlock_t power_lock;		/* must be held when changing a power
					   manager register */
	spinlock_t pll_lock;		/* must be held when changing a PLL
					   control register */
};

/**
 * struct jz47xx_clk - private data for a clock
 * @hw: see Documentation/clk.txt
 * @cgu: a pointer to the CGU data
 * @idx: the index of this clock in cgu->clock_info
 */
struct jz47xx_clk {
	struct clk_hw hw;
	struct jz47xx_cgu *cgu;
	unsigned idx;
};

#define to_jz47xx_clk(_hw) container_of(_hw, struct jz47xx_clk, hw)

/**
 * jz47xx_cgu_new - create a new CGU instance
 * @clock_info: an array of clock information structures describing the clocks
 *              which are implemented by the CGU
 * @num_clocks: the number of entries in clock_info
 * @np: the device tree node which causes this CGU to be probed
 *
 * Returns an opaque pointer to the CGU instance if initialisation & clock
 * registration is successful, otherwise NULL.
 */
struct jz47xx_cgu *jz47xx_cgu_new(const struct jz47xx_cgu_clk_info *clock_info,
				  unsigned num_clocks,
				  struct device_node *np);

/**
 * jz47xx_cgu_register_clocks - Registers the clocks
 * @cgu: pointer to cgu data
 *
 * Returns 1 on success and -EINVAL if unsuccesful.
 */
int jz47xx_cgu_register_clocks(struct jz47xx_cgu *cgu);

#endif /* __DRIVERS_CLK_JZ47XX_JZ47XX_CGU_H__ */

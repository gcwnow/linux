/*
 * JZ4740 SoC TCU clocks driver
 *
 * Copyright (c) 2015 Paul Cercueil <paul@crapouillou.net>
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

#ifndef __LINUX_CLK_JZ4740_TCU_H_
#define __LINUX_CLK_JZ4740_TCU_H_

#include <linux/types.h>

struct clk;

u16 jz4740_tcu_read_tcsr(struct clk *timer);
void jz4740_tcu_write_tcsr(struct clk *timer, u16 mask, u16 value);

#endif /* __LINUX_CLK_JZ4740_TCU_H_ */

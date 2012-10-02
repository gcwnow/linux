/*
 * linux/include/asm-mips/mach-jz4770/jz4770misc.h
 *
 * JZ4770 misc definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4770MISC_H__
#define __JZ4770MISC_H__

#define REG32(addr)	*((volatile unsigned int *)(addr))
#define REG16(addr)	*((volatile unsigned short *)(addr))
#define REG8(addr)	*((volatile unsigned char *)(addr))

#endif /* __JZ4770MISC_H__ */

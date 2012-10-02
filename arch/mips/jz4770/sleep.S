/*
 * linux/arch/mips/jz4770/sleep.S
 *
 * jz4770 Assembler Sleep/WakeUp Management Routines
 *
 * Copyright (C) 2005 - 2010 Ingenic Semiconductor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define __MIPS_ASSEMBLER

#include <asm/regdef.h>
#include <asm/mipsregs.h>
#include <asm/cacheops.h>

#include <asm/mach-jz4770/jz4770cpm.h>
#inclede <asm/mach-jz4770/jz4770emc.h>

#define K0BASE 0x80000000
#define CFG_DCACHE_SIZE  16384
#define CFG_ICACHE_SIZE  16384
#define CFG_CACHELINE_SIZE	32
/**********************************************************/
	.data
	.align	4
	.set	noreorder
	.set	noat

reg_save:
	.space	70*4
/*********************************************************/
	.text
	.extern jz_flush_cache_all

/*
 * jz_cpu_sleep()
 *
 * Forces CPU into sleep mode,and we will power down p0 in this mode!
 */

	.align 5
	.globl	jz_cpu_sleep
jz_cpu_sleep:

	/* save hi, lo and general registers except k0($26) and k1($27) (total 32) */
	la	k0,	reg_save
	mfhi	k1
	sw	$0,	0(k0)
	sw	$1,  	4(k0)
	sw	k1,	120(k0)
	mflo	k1
	sw	$2,	8(k0)
	sw	$3,  	12(k0)
	sw	k1,	124(k0)
	sw	$4,  	16(k0)
	sw	$5,  	20(k0)
	sw	$6,  	24(k0)
	sw	$7,  	28(k0)
	sw	$8,  	32(k0)
	sw	$9,  	36(k0)
	sw	$10, 	40(k0)
	sw	$11, 	44(k0)
	sw	$12, 	48(k0)
	sw	$13, 	52(k0)
	sw	$14, 	56(k0)
	sw	$15, 	60(k0)
	sw	$16, 	64(k0)
	sw	$17, 	68(k0)
	sw	$18, 	72(k0)
	sw	$19, 	76(k0)
	sw	$20, 	80(k0)
	sw	$21, 	84(k0)
	sw	$22, 	88(k0)
	sw	$23, 	92(k0)
	sw	$24, 	96(k0)
	sw	$25, 	100(k0)
	sw	$28, 	104(k0)
	sw	$29, 	108(k0)		/* saved sp */
	sw	$30, 	112(k0)
	sw	$31, 	116(k0)		/* saved ra */

	/* save CP0 registers total 31*/
	mfc0	$1, $0
	mfc0	$2, $1
	mfc0	$3, $2
	mfc0	$4, $3
	mfc0	$5, $4
	mfc0	$6, $5
	mfc0	$7, $6
	mfc0	$8, $8
	mfc0	$9, $10
	mfc0	$10,$12
	mfc0	$11, $12,1
	mfc0	$12, $13
	mfc0	$13, $14
	mfc0	$14, $15
	mfc0	$15, $15,1
	mfc0	$16, $16
	mfc0	$17, $16,1
	mfc0	$18, $16,2
	mfc0	$19, $16,3
	mfc0	$20, $16, 7
	mfc0	$21, $17

	sw	$1,  128(k0)
	sw	$2,  132(k0)
	sw	$3,  136(k0)
	sw	$4,  140(k0)
	sw	$5,  144(k0)
	sw	$6,  148(k0)
	sw	$7,  152(k0)
	sw	$8,  156(k0)
	sw	$9,  160(k0)
	sw	$10, 164(k0)
	sw	$11, 168(k0)
	sw	$12, 172(k0)
	sw	$13, 176(k0)
	sw	$14, 180(k0)
	sw	$15, 184(k0)
	sw	$16, 188(k0)
	sw	$17, 192(k0)
	sw	$18, 196(k0)
	sw	$19, 200(k0)
	sw	$20, 204(k0)
	sw	$21, 208(k0)

	mfc0	$1, $18
	mfc0	$2, $19
	mfc0	$3, $23
	mfc0	$4, $24
	mfc0	$5, $26
	mfc0	$6, $28
	mfc0	$7, $28,1
	mfc0	$8, $30
	mfc0	$9, $31
	mfc0	$10,$5,4      /*save big page mode register*/

	sw	$1,  212(k0)
	sw	$2,  216(k0)
	sw	$3,  220(k0)
	sw	$4,  224(k0)
	sw	$5,  228(k0)
	sw	$6,  232(k0)
	sw	$7,  236(k0)
	sw	$8,  240(k0)
	sw	$9,  244(k0)
	sw	$10, 248(k0)

	/* preserve virtual address of stack */
	la	k0, sleep_save_sp
	sw	sp, 0(k0)

	/* flush caches and write buffers */
	jal	jz_flush_cache_all
	nop

	/* load ins to icache */
	la      $29, icache_data_start
	la      $9,icache_date_end
	sub     $29,$29,32
	add     $9,$9,32
	la      $8,~(31)
	and     $29,$29,$8
	and     $9,$9,$8
xfer_d2i:
	cache   0x1c, 0x0($29)
	addiu   $29,$29,32
	bne     $29,$9,xfer_d2i
	nop
	nop

	/* enter sleep mode */
	.align 5
	.set mips3
icache_data_start:
	sync
	sync
	wait
	nop
	nop
	.set mips32
	nop
9:	j 9b
	nop
	nop
icache_date_end:

/*
 * jz_cpu_resume()
 *
 * entry point from bootloader into kernel during resume
 */

	.align 5
	.globl	jz_cpu_resume
jz_cpu_resume:
	la	$28,CPM_CPPSR
	li	$6, 0x00000001
	la	$29,CPM_CPCCR
	li	$8, 0xfff0f0ff
	li	$7, 0x00430500
	lw      $9, 0x0($29)
	and	$9, $9, $8
	or	$9, $9, $7

	.align 5
	sw      $6, 0x0($28)
	sw      $9, 0x0($29)
	nop
	nop

	/* Init caches */
	.set	mips32
	mtc0	zero, CP0_TAGLO

	li	t0, K0BASE
	ori	t1, t0, CFG_DCACHE_SIZE

1:
	cache	Index_Store_Tag_D, 0(t0)
	bne	t0, t1, 1b
	addiu	t0, t0, CFG_CACHELINE_SIZE

	li	t0, K0BASE
	ori	t1, t0, CFG_ICACHE_SIZE
2:
	cache	Index_Store_Tag_I, 0(t0)
	bne	t0, t1, 2b
	addiu	t0, t0, CFG_CACHELINE_SIZE

	/* Invalidate BTB */
	mfc0	t0, CP0_CONFIG, 7
	nop
	ori	t0, 2
	mtc0	t0, CP0_CONFIG, 7
	nop

	/* restore saved sp */
	la	t0, sleep_save_sp
	lw	sp, 0(t0)

	/* restore CP0 registers(total 26) */
	la	k0,	reg_save

	lw	$1,  128(k0)
	lw	$2,  132(k0)
	lw	$3,  136(k0)
	lw	$4,  140(k0)
	lw	$5,  144(k0)
	lw	$6,  148(k0)
	lw	$7,  152(k0)
	lw	$8,  156(k0)
	lw	$9,  160(k0)
	lw	$10, 164(k0)
	lw	$11, 168(k0)
	lw	$12, 172(k0)
	lw	$13, 176(k0)
	lw	$14, 180(k0)
	lw	$15, 184(k0)
	lw	$16, 188(k0)
	lw	$17, 192(k0)
	lw	$18, 196(k0)
	lw	$19, 200(k0)
	lw	$20, 204(k0)
	lw	$21, 208(k0)

	mtc0	$1, $0
	mtc0	$2, $1
	mtc0	$3, $2
	mtc0	$4, $3
	mtc0	$5, $4
	mtc0	$6, $5
	mtc0	$7, $6
	mtc0	$8, $8
	mtc0	$9, $10
	mtc0	$10,$12
	mtc0	$11, $12,1
	mtc0	$12, $13
	mtc0	$13, $14
	mtc0	$14, $15
	mtc0	$15, $15,1
	mtc0	$16, $16
	mtc0	$17, $16,1
	mtc0	$18, $16,2
	mtc0	$19, $16,3
	mtc0	$20, $16,7
	mtc0	$21, $17


	lw	$1,  212(k0)
	lw	$2,  216(k0)
	lw	$3,  220(k0)
	lw	$4,  224(k0)
	lw	$5,  228(k0)
	lw	$6,  232(k0)
	lw	$7,  236(k0)
	lw	$8,  240(k0)
	lw	$9,  244(k0)
	lw	$10, 248(k0)

	mtc0	$1, $18
	mtc0	$2, $19
	mtc0	$3, $23
	mtc0	$4, $24
	mtc0	$5, $26
	mtc0	$6, $28
	mtc0	$7, $28,1
	mtc0	$8, $30
	mtc0	$9, $31
	mtc0	$10,$5,4  /*restore big page register*/

	/*Restore cpu registers*/
	lw	k1,	120(k0)		/* hi */
	lw	$0,	0(k0)
	lw	$1,  	4(k0)
	mthi	k1
	lw	k1,	124(k0)		/* lo */
	lw	$2,	8(k0)
	lw	$3,  	12(k0)
	mtlo	k1
	lw	$4,  	16(k0)
	lw	$5,  	20(k0)
	lw	$6,  	24(k0)
	lw	$7,  	28(k0)
	lw	$8,  	32(k0)
	lw	$9,  	36(k0)
	lw	$10, 	40(k0)
	lw	$11, 	44(k0)
	lw	$12, 	48(k0)
	lw	$13, 	52(k0)
	lw	$14, 	56(k0)
	lw	$15, 	60(k0)
	lw	$16, 	64(k0)
	lw	$17, 	68(k0)
	lw	$18, 	72(k0)
	lw	$19, 	76(k0)
	lw	$20, 	80(k0)
	lw	$21, 	84(k0)
	lw	$22, 	88(k0)
	lw	$23, 	92(k0)
	lw	$24, 	96(k0)
	lw	$25, 	100(k0)
	lw	$28, 	104(k0)
	lw	$29, 	108(k0)		/* restore sp */
	lw	$30, 	112(k0)
	lw	$31, 	116(k0)		/* restore ra */

	/* return to caller */


	jr	ra
	nop
	nop
	nop

sleep_save_sp:
	.word	0			/* preserve sp here */

	.set	reorder

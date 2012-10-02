/*
 * linux/include/asm-mips/mach-jz4770/jz4770intc.h
 *
 * JZ4770 INTC register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4770INTC_H__
#define __JZ4770INTC_H__

#include <asm/mach-jz4770/jz4770dmac.h>
#include <asm/mach-jz4770/jz4770misc.h>
#include <asm/mach-jz4770/jz4770sadc.h>


#define	INTC_BASE	0xB0001000

/*************************************************************************
 * INTC (Interrupt Controller)
 *************************************************************************/
/* n = 0 ~ 1 */
#define INTC_ISR(n)	(INTC_BASE + 0x00 + (n) * 0x20)
#define INTC_IMR(n)	(INTC_BASE + 0x04 + (n) * 0x20)
#define INTC_ICMR(n)    INTC_IMR(n)
#define INTC_IMSR(n)	(INTC_BASE + 0x08 + (n) * 0x20)
#define INTC_ICMSR(n)   INTC_IMSR(n)
#define INTC_IMCR(n)	(INTC_BASE + 0x0c + (n) * 0x20)
#define INTC_ICMCR(n)	INTC_IMCR(n)
#define INTC_IPR(n)	(INTC_BASE + 0x10 + (n) * 0x20)
//#define INTC_ISSR	(INTC_BASE + 0x18)  /* Interrupt Controller Source Set Register */
//#define INTC_ISCR	(INTC_BASE + 0x1c)  /* Interrupt Controller Source Clear Register */

#define REG_INTC_ISR(n)		REG32(INTC_ISR(n))
#define REG_INTC_IMR(n)		REG32(INTC_IMR(n))
#define REG_INTC_IMSR(n)	REG32(INTC_IMSR(n))
#define REG_INTC_IMCR(n)	REG32(INTC_IMCR(n))
#define REG_INTC_IPR(n)		REG32(INTC_IPR(n))
//#define REG_INTC_ISSR   REG32(INTC_ISSR)
//#define REG_INTC_ISCR   REG32(INTC_ISCR)

// 1st-level interrupts
#define IRQ_I2C1	0
#define IRQ_I2C0	1
#define IRQ_UART3	2
#define IRQ_UART2	3
#define IRQ_UART1	4
#define IRQ_UART0	5
#define IRQ_GPU		6
#define IRQ_SSI1   	7
#define IRQ_SSI0   	8
#define IRQ_TSSI	9
#define IRQ_BDMA	10
#define IRQ_KBC		11
#define IRQ_GPIO5	12
#define IRQ_GPIO4	13
#define IRQ_GPIO3	14
#define IRQ_GPIO2	15
#define IRQ_GPIO1	16
#define IRQ_GPIO0	17
#define IRQ_SADC	18
#define IRQ_ETH		19
#define IRQ_UHC		20
#define IRQ_OTG		21
//#define IRQ_MDMA	22
#define IRQ_I2C2	22
#define IRQ_DMAC1	23
#define IRQ_DMAC0	24
#define IRQ_TCU2	25
#define IRQ_TCU1	26
#define IRQ_TCU0	27
#define IRQ_GPS		28
#define IRQ_IPU		29
#define IRQ_CIM		30
#define IRQ_LCD		31

#define IRQ_RTC		32
#define IRQ_OWI		33
#define IRQ_AIC 	34
#define IRQ_MSC2	35
#define IRQ_MSC1	36
#define IRQ_MSC0	37
#define IRQ_SCC		38
#define IRQ_BCH		39
#define IRQ_PCM0	40
#define IRQ_PCM1	41
#define IRQ_UART4	42
#define IRQ_AOSD        43
#define IRQ_HARB2	44
#define IRQ_I2S2	45
#define IRQ_CPM		47

// 2nd-level interrupts

#define IRQ_DMA_0	64    /* 64,65,66,67,68,69 */
#define IRQ_DMA_1	(IRQ_DMA_0 + HALF_DMA_NUM) /* 70,71,72,73,74,75 */
#define IRQ_MDMA_0	(IRQ_DMA_0 + MAX_DMA_NUM)   /* 76,77,78 */
#define IRQ_BDMA_0	(IRQ_MDMA_0 + MAX_MDMA_NUM) /* 79,80,81 */
#define IRQ_SADC_BASE	(IRQ_BDMA_0 + MAX_BDMA_NUM) /* 82-87 */

#define IRQ_GPIO_0	(IRQ_SADC_BASE + SADC_IRQ_NUM)

#define NUM_INTC	48
#define NUM_DMA         MAX_DMA_NUM	/* 12 */
#define NUM_MDMA        MAX_MDMA_NUM	/* 3 */
#define NUM_GPIO        MAX_GPIO_NUM	/* GPIO NUM: 192, Jz4770 real num GPIO 178 */


#ifndef __MIPS_ASSEMBLER


/***************************************************************************
 * INTC
 ***************************************************************************/
#define __intc_unmask_irq(n)	(REG_INTC_IMCR((n)/32) = (1 << ((n)%32)))
#define __intc_mask_irq(n)	(REG_INTC_IMSR((n)/32) = (1 << ((n)%32)))
#define __intc_ack_irq(n)	(REG_INTC_IPR((n)/32) = (1 << ((n)%32))) /* A dummy ack, as the Pending Register is Read Only. Should we remove __intc_ack_irq() */


#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4770INTC_H__ */


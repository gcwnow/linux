

#ifndef __CHIP_AUX_H__
#define __CHIP_AUX_H__

#define AUX_BASE 		(0xb32a0000)
#define AUX_CTRL 		(AUX_BASE +0x0)
#define AUX_SPINLK  		(AUX_BASE +0x4)
#define AUX_SPIN1  		(AUX_BASE +0x8)
#define AUX_SPIN2  		(AUX_BASE +0xc)
#define AUX_MIRQP 		(AUX_BASE +0x10)
#define AUX_MESG  		(AUX_BASE +0x14)
#define CORE_MIRQP 		(AUX_BASE +0x18)
#define CORE_MESG  		(AUX_BASE +0x1c)

#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/asm.h>
#include <asm/errno.h>
#include <asm/fpregdef.h>
#include <asm/mipsregs.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>


void jz4770_fpu_dump(void)
{
	unsigned int fgpr[32];
	unsigned int fscr, fir, i;

	for (i = 0; i < 32; i ++)
		fgpr[i] = 0xdeadbeef;

	__asm__ __volatile__(
		"swc1 $f0, 0(%0)\n\t"
		"swc1 $f1, 4(%0)\n\t"
		"swc1 $f2, 8(%0)\n\t"
		"swc1 $f3, 12(%0)\n\t"
		"swc1 $f4, 16(%0)\n\t"
		"swc1 $f4, 20(%0)\n\t"
		"swc1 $f6, 24(%0)\n\t"
		"swc1 $f6, 28(%0)\n\t"
		"swc1 $f8, 32(%0)\n\t"
		"swc1 $f9, 36(%0)\n\t"
		"swc1 $f10, 40(%0)\n\t"
		"swc1 $f11, 44(%0)\n\t"
		"swc1 $f12, 48(%0)\n\t"
		"swc1 $f13, 52(%0)\n\t"
		"swc1 $f14, 56(%0)\n\t"
		"swc1 $f15, 60(%0)\n\t"
		"swc1 $f16, 64(%0)\n\t"
		"swc1 $f17, 68(%0)\n\t"
		"swc1 $f18, 72(%0)\n\t"
		"swc1 $f19, 76(%0)\n\t"
		"swc1 $f20, 80(%0)\n\t"
		"swc1 $f21, 84(%0)\n\t"
		"swc1 $f22, 88(%0)\n\t"
		"swc1 $f23, 92(%0)\n\t"
		"swc1 $f24, 96(%0)\n\t"
		"swc1 $f25, 100(%0)\n\t"
		"swc1 $f26, 104(%0)\n\t"
		"swc1 $f27, 108(%0)\n\t"
		"swc1 $f28, 112(%0)\n\t"
		"swc1 $f29, 116(%0)\n\t"
		"swc1 $f30, 120(%0)\n\t"
		"swc1 $f31, 124(%0)\n\t"
		:
		:"r"(fgpr)
	);

	/* Get FPU control and status register */
	__asm__ __volatile__(
		"cfc1	%0, $31 \n\t"
		:"=r"(fscr)
		:
	);

	/* Get FPU implementation and revision register */
	__asm__ __volatile__(
		"cfc1	%0, $0 \n\t"
		:"=r"(fir)
		:
	);

	printk("Dump of FPU: \n");

	for (i = 0; i < 32; i += 4)
		printk("%08x %08x %08x %08x \n",
		       fgpr[i],fgpr[i+1],fgpr[i+2],fgpr[i+3]);
	printk("FPU control and status register          = %08x \n",fscr);
	printk("FPU implementation and revision register = %08x \n",fir);

	fscr |= 0x11;

	__asm__ __volatile__(
		"ctc1	%0, $31 \n\t"
		:
		:"r"(fscr)
	);
	__asm__ __volatile__(
		"cfc1	%0, $31 \n\t"
		:"=r"(fscr)
		:
	);
	printk("FPU control and status register          = %08x \n",fscr);
}

void jz4770_fpu_init(unsigned int round)
{
	unsigned int tmp;

	__asm__ __volatile__(
		"cfc1	%0, $31 \n\t"
		:"=r"(tmp)
		:
	);

//	printk("original jz4770 FCSR value: %8x\n",tmp);

	tmp = 0x0;
	/* Jz4770 FPU init */
	/* disable any exception */
	tmp |= (0x0 << 7);
	/* Set ronud mode */
	tmp &= ~0x3;
	tmp |= (round & 0x3);

//	printk("plan to set jz4770 FPU FCSR to %08x \n",tmp);
	__asm__ __volatile__(
		"ctc1	%0, $28 \n\t"
		:
		:"r"(tmp)
	);

	__asm__ __volatile__(
		"cfc1	%0, $31 \n\t"
		:"=r"(tmp)
		:
	);

//	printk("jz4770 FCSR value after setting: %8x\n",tmp);

}

EXPORT_SYMBOL(jz4770_fpu_init);

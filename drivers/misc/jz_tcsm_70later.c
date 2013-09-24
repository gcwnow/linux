/*
 * linux/drivers/misc/tcsm.c
 *
 * Virtual device driver with tricky appoach to manage TCSM
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#if defined(ANDROID)
#include <linux/wakelock.h>
#endif

#include <asm/pgtable.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>

#include <asm/irq.h>
#include <asm/thread_info.h>
#include <asm/uaccess.h>

#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770aux.h>
#include <asm/mach-jz4770/irq.h>

#include <linux/syscalls.h>

#include "jz_tcsm.h"

MODULE_LICENSE("GPL");

/*
 * fops routines
 */

static int tcsm_open(struct inode *inode, struct file *filp);
static int tcsm_release(struct inode *inode, struct file *filp);
static ssize_t tcsm_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t tcsm_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static long tcsm_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);
static int tcsm_mmap(struct file *file, struct vm_area_struct *vma);
static struct file_operations tcsm_fops =
{
open:		tcsm_open,
		release:	tcsm_release,
		read:		tcsm_read,
		write:		tcsm_write,
		unlocked_ioctl:		tcsm_ioctl,
		mmap:           tcsm_mmap,
};

#if defined(ANDROID)
static struct wake_lock tcsm_wake_lock;
#endif

static struct completion tcsm_comp;
static struct tcsm_sem tcsm_sem;

static void tcsm_sem_init(struct tcsm_sem *tcsm_sem)
{
	sema_init(&(tcsm_sem->sem),1);
	tcsm_sem->tcsm_file_mode_pre = R_W;
}

static long tcsm_on(struct file_info *file_info)
{
	struct pt_regs *info = task_pt_regs(current);
	unsigned int dat;

	tcsm_sem.owner_pid = current->pid;

#if 0 /* For some reason turning this bit off again causes a page fault in the kernel */
	if(INREG32(CPM_OPCR) & OPCR_IDLE_DIS) {
		return -EBUSY;
	}
#endif
        SETREG32(CPM_OPCR, OPCR_IDLE_DIS);

	dat = INREG32(CPM_CLKGR1);

	dat &= ~(CLKGR1_AUX | CLKGR1_VPU | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME);

	OUTREG32(CPM_CLKGR1,dat);
#if 0 /* enabled by default in pm.c right now */
        CLRREG32(CPM_LCR, LCR_PDAHB1); /* enable power to AHB1 and VPU */
#endif
	SETREG32(CPM_CLKGR1,CLKGR1_ME); //no use me

	__asm__ __volatile__ (
			"mfc0  $2, $16,  7   \n\t"
			"ori   $2, $2, 0x340 \n\t"
			"andi  $2, $2, 0x3ff \n\t"
			"mtc0  $2, $16,  7  \n\t"
			"nop                  \n\t");
	enable_irq(IRQ_VPU);
	file_info->is_on = 1;

	dbg_tcsm("Tcsm[%d:%d] on\n", current->tgid, current->pid);
	printk("cp0 status=0x%08x\n", (unsigned int)info->cp0_status);
#if defined(ANDROID)
	wake_lock(&tcsm_wake_lock);
#endif
	return 0;
}

static long tcsm_off(struct file_info *file_info)
{
	unsigned int dat = 0;

#if 0 /* enabled by default in pm.c right now */
        SETREG32(CPM_LCR, LCR_PDAHB1);
#endif

	disable_irq_nosync(IRQ_VPU);
	dat |= (CLKGR1_AUX | CLKGR1_VPU | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME);

	printk("dat = 0x%08x\n",dat);

	OUTREG32(CPM_CLKGR1,dat);

	__asm__ __volatile__ (
			"mfc0  $2, $16,  7   \n\t"
			"andi  $2, $2, 0xbf \n\t"
			"mtc0  $2, $16,  7  \n\t"
			"nop                  \n\t");
#if 0 /* For some reason turning this bit off here causes a pagefault in the kernel */
	CLRREG32(CPM_OPCR, OPCR_IDLE_DIS);
#endif

#if defined(ANDROID)
	wake_unlock(&tcsm_wake_lock);
#endif
	file_info->is_on = 0;
//	up(&tcsm_sem.sem);
	tcsm_sem.owner_pid = 0;

	dbg_tcsm("Tcsm[%d:%d] off\n", current->tgid, current->pid);
	return 0;
}

static int tcsm_open(struct inode *inode, struct file *filp)
{
	struct file_info *file_info;

	file_info = kzalloc(sizeof(struct file_info),GFP_KERNEL);
	filp->private_data = file_info;
	dbg_tcsm("Tcsm[%d:%d] open\n", current->tgid, current->pid);
	return 0;
}

static int tcsm_release(struct inode *inode, struct file *filp)
{
	struct file_info *file_info = filp->private_data;

	dbg_tcsm("Tcsm[%d:%d] close\n", current->tgid, current->pid);
	if (file_info->is_on) {
		printk("Tcsm[%d:%d] tcsm was closed without turning it off, forcing it off\n", current->tgid, current->pid);
                tcsm_off(file_info);
	}
	kfree(file_info);
	up(&tcsm_sem.sem);
	return 0;
}

static ssize_t tcsm_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	printk("tcsm: read is not implemented\n");
	return -1;
}

static ssize_t tcsm_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	printk("tcsm: write is not implemented\n");
	return -1;
}

static long tcsm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct file_info *file_info = filp->private_data;
	spinlock_t ioctl_lock = file_info->ioctl_lock;
	unsigned long flags;

	spin_lock_irqsave(&ioctl_lock, flags);
	switch (cmd) {
	case TCSM_TOCTL_WAIT_COMPLETE:
		dbg_tcsm("Tcsm[%d:%d] ioctl:TCSM_TOCTL_WAIT_COMPLETE\n", current->tgid, current->pid);
		if (file_info->is_on) {
			spin_unlock_irqrestore(&ioctl_lock, flags);
			ret = wait_for_completion_interruptible_timeout(&tcsm_comp,msecs_to_jiffies(arg));
			spin_lock_irqsave(&ioctl_lock, flags);
		} else {
			printk("Tcsm[%d:%d]: Please turn on tcsm before waiting completion.\n", current->tgid, current->pid);
			ret = -1;
		}
		break;
	case TCSM_TOCTL_PREPARE_DIR:
		dbg_tcsm("Tcsm[%d:%d] ioctl:TCSM_TOCTL_PREPARE_DIR\n", current->tgid, current->pid);
		if (tcsm_sem.owner_pid == current->pid) {
			printk("In %s:%s-->pid[%d]:tid[%d] can't turn on tcsm twice!\n", __FILE__, __func__, current->tgid, current->pid);
			ret = -1;
			break;
		}

		if (down_trylock(&tcsm_sem.sem)) {
			dbg_tcsm("Tcsm[%d:%d] Return Directly\n", current->tgid, current->pid);
			ret = -1;
		} else
			ret = tcsm_on(file_info);
		break;
	case TCSM_TOCTL_PREPARE_BLOCK:
		dbg_tcsm("Tcsm[%d:%d] ioctl:TCSM_TOCTL_PREPARE_BLOCK\n", current->tgid, current->pid);
		if (tcsm_sem.owner_pid == current->pid) {
			printk("In %s:%s-->pid[%d]:tid[%d] can't turn on tcsm twice!\n", __FILE__, __func__, current->tgid, current->pid);
			ret = -1;
			break;
		}

		if (down_trylock(&tcsm_sem.sem)) {
			dbg_tcsm("Tcsm[%d:%d] Block\n", current->tgid, current->pid);
			spin_unlock_irqrestore(&ioctl_lock, flags);
			if (down_interruptible(&tcsm_sem.sem) != 0) {
				dbg_tcsm("Tcsm[%d:%d] down error!\n", current->tgid, current->pid);
				ret = -1;
				return ret;
			} else {
				spin_lock_irqsave(&ioctl_lock, flags);
				ret = tcsm_on(file_info);
			}
		} else
			ret = tcsm_on(file_info);
		break;
	case TCSM_TOCTL_FLUSH_WORK:
		dbg_tcsm("Tcsm[%d:%d] ioctl:TCSM_TOCTL_FLUSH_WORK\n", current->tgid, current->pid);
		if (file_info->is_on)
			tcsm_off(file_info);
		else {
			printk("Tcsm[%d:%d] Please turn on tcsm before flush work.\n", current->tgid, current->pid);
			ret = -1;
		}
		break;
	default:
		printk(KERN_ERR "%s:cmd(0x%x) error !!!",__func__,cmd);
		ret = -1;
	}
	spin_unlock_irqrestore(&ioctl_lock, flags);
	return ret;
}

static int tcsm_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */
	if (io_remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff, vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static struct miscdevice tcsm_dev = {
	TCSM_MINOR,
	"tcsm",
	&tcsm_fops
};


/*
 * Module init and exit
 */

#if defined(CONFIG_MACH_JZ4770)
static irqreturn_t vpu_interrupt(int irq, void *dev)
{
	CLRREG32(AUX_MIRQP, 0x1);
	complete(&tcsm_comp);
	return IRQ_HANDLED;
}
#endif

static int __init tcsm_init(void)
{
	int ret;

	ret = misc_register(&tcsm_dev);
	if (ret < 0) {
		return ret;
	}
#if defined(ANDROID)
	wake_lock_init(&tcsm_wake_lock, WAKE_LOCK_SUSPEND, "tcsm");
#endif

	tcsm_sem_init(&tcsm_sem);

#if defined(CONFIG_MACH_JZ4770)
	init_completion(&tcsm_comp);
    request_irq(IRQ_VPU,vpu_interrupt,IRQF_DISABLED,"vpu",NULL);
    disable_irq_nosync(IRQ_VPU);
#endif
	printk("Virtual Driver of JZ TCSM registered\n");
	return 0;
}

static void __exit tcsm_exit(void)
{
	misc_deregister(&tcsm_dev);
#if defined(ANDROID)
	wake_lock_destroy(&tcsm_wake_lock);
#endif
#if defined(CONFIG_MACH_JZ4770)
	free_irq(IRQ_VPU,NULL);
#endif
}

module_init(tcsm_init);
module_exit(tcsm_exit);

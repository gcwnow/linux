/*
 * linux/drivers/misc/jz_vpu.c
 *
 * Virtual device driver to manage VPU for JZ4770.
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 * Copyright (C) 2013  Wladimir J. van der Laan
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
#include <linux/list.h>

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

#include <uapi/video/jz_vpu.h>

MODULE_LICENSE("GPL");

#ifdef JZ_VPU_DEBUG
#define dbg_jz_vpu(x...) printk(x)
#else
#define dbg_jz_vpu(x...)
#endif

/* Physical memory heap structure */
struct jz_vpu_mem {
	struct page *page;
	unsigned long physical;
	unsigned long kaddr;
	size_t size;
        struct list_head list;
};

/* Open VPU connection info */
struct file_info {
	/* mutex to protect structure */
	struct semaphore mutex;
	/* completion ioctl lock */
	spinlock_t ioctl_lock;
	/* memory allocations belonging to this VPU connection */
        struct list_head mem_list;
};

/*
 * fops routines
 */

static int jz_vpu_open(struct inode *inode, struct file *filp);
static int jz_vpu_release(struct inode *inode, struct file *filp);
static ssize_t jz_vpu_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t jz_vpu_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static long jz_vpu_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);
static int jz_vpu_mmap(struct file *file, struct vm_area_struct *vma);

static struct file_operations jz_vpu_fops =
{
	open:		jz_vpu_open,
	release:	jz_vpu_release,
	read:		jz_vpu_read,
	write:		jz_vpu_write,
	unlocked_ioctl:		jz_vpu_ioctl,
	mmap:		jz_vpu_mmap,
};

static struct completion jz_vpu_comp;

static long jz_vpu_on(struct file_info *file_info)
{
#ifdef JZ_VPU_DEBUG
	struct pt_regs *info = task_pt_regs(current);
#endif
	unsigned int dat;

	if(INREG32(CPM_OPCR) & OPCR_IDLE_DIS) {
		/* VPU already turned on by another process */
		return -EBUSY;
	}
        SETREG32(CPM_OPCR, OPCR_IDLE_DIS);

	dat = INREG32(CPM_CLKGR1);

	dat &= ~(CLKGR1_AUX | CLKGR1_VPU | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME);

	OUTREG32(CPM_CLKGR1,dat);

	/* enable power to AHB1 (VPU), then wait for it to enable */
	CLRREG32(CPM_LCR, LCR_PDAHB1);
	while(!(REG_CPM_LCR && LCR_PDAHB1S)) ;

	SETREG32(CPM_CLKGR1,CLKGR1_ME); /* no use for ME */

        /* Enable partial kernel mode. This allows user space access
         * to the TCSM, cache instructions and VPU. */
	__asm__ __volatile__ (
			"mfc0  $2, $16,  7   \n\t"
			"ori   $2, $2, 0x340 \n\t"
			"andi  $2, $2, 0x3ff \n\t"
			"mtc0  $2, $16,  7  \n\t"
			"nop                  \n\t");
	enable_irq(IRQ_VPU);

	dbg_jz_vpu("jz-vpu[%d:%d] on\n", current->tgid, current->pid);
	dbg_jz_vpu("cp0 status=0x%08x\n", (unsigned int)info->cp0_status);
	return 0;
}

static long jz_vpu_off(struct file_info *file_info)
{
	unsigned int dat = 0;

        /* Power down AHB1 (VPU) */
        SETREG32(CPM_LCR, LCR_PDAHB1);
        while(!(REG_CPM_LCR && LCR_PDAHB1S)) ;

	disable_irq_nosync(IRQ_VPU);

	dat |= (CLKGR1_AUX | CLKGR1_VPU | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME);
	OUTREG32(CPM_CLKGR1,dat);

        /* Disable partial kernel mode. This disallows user space access
         * to the TCSM, cache instructions and VPU. */
	__asm__ __volatile__ (
			"mfc0  $2, $16,  7   \n\t"
			"andi  $2, $2, 0xbf \n\t"
			"mtc0  $2, $16,  7  \n\t"
			"nop                  \n\t");

        CLRREG32(CPM_OPCR, OPCR_IDLE_DIS);

	dbg_jz_vpu("jz-vpu[%d:%d] off\n", current->tgid, current->pid);
	return 0;
}

/* Allocate a new contiguous memory block, return the physical address
 * that can be mmapped. */
static unsigned long jz_vpu_alloc_phys(struct file_info *file_info, size_t size, unsigned long *physical)
{
        struct jz_vpu_mem *mem;

	mem = kmalloc(sizeof(struct jz_vpu_mem), GFP_KERNEL);
	if(mem == NULL)
		return -ENOMEM;
	INIT_LIST_HEAD(&mem->list);

	mem->size = size;
	mem->page = alloc_pages(GFP_KERNEL | __GFP_NOWARN, get_order(mem->size));
	if(mem->page == NULL)
	{
		kfree(mem);
		return -ENOMEM;
	}
	mem->kaddr = (unsigned long)page_address(mem->page);
	mem->physical = page_to_phys(mem->page);

	down(&file_info->mutex);
	list_add_tail(&mem->list, &file_info->mem_list);
	up(&file_info->mutex);

	*physical = mem->physical;
	return 0;
}

/* Free one contiguous memory block by pointer */
static int jz_vpu_free_mem(struct file_info *file_info, struct jz_vpu_mem *mem)
{
	printk(KERN_ERR "jz-vpu[%d:%d] free mem %p %08x size=%i\n", current->tgid, current->pid, 
			mem, (unsigned int)mem->physical, (unsigned int)mem->size);

	down(&file_info->mutex);
	list_del(&mem->list);
	up(&file_info->mutex);

	free_pages(mem->kaddr, get_order(mem->size));
	kfree(mem);
	return 0;
}

/* Free one contiguous memory block by physical address */
static int jz_vpu_free_phys(struct file_info *file_info, unsigned long physical)
{
	struct jz_vpu_mem *mem;
	list_for_each_entry(mem, &file_info->mem_list, list)
	{
		if(mem->physical == physical)
		{
			jz_vpu_free_mem(file_info, mem);
			return 0;
		}
	}
	printk(KERN_ERR "jz-vpu[%d:%d] attempt to free non-allocated memory %08x\n", current->tgid, current->pid, (unsigned int)physical);
	return -ENOENT;
}

static int jz_vpu_open(struct inode *inode, struct file *filp)
{
	struct file_info *file_info;
        int ret;

	file_info = kzalloc(sizeof(struct file_info),GFP_KERNEL);
	filp->private_data = file_info;
	INIT_LIST_HEAD(&file_info->mem_list);
	dbg_jz_vpu("jz-vpu[%d:%d] open\n", current->tgid, current->pid);

        ret = jz_vpu_on(file_info);
	if(ret < 0)
	{
		kfree(file_info);
	}
	sema_init(&file_info->mutex, 1);
	return ret;
}

static int jz_vpu_release(struct inode *inode, struct file *filp)
{
	struct file_info *file_info = filp->private_data;
	struct jz_vpu_mem *mem, *next;

	dbg_jz_vpu("jz-vpu[%d:%d] close\n", current->tgid, current->pid);
        jz_vpu_off(file_info);

	/* Free all contiguous memory blocks associated with this VPU connection */
	list_for_each_entry_safe(mem, next, &file_info->mem_list, list)
	{
		jz_vpu_free_mem(file_info, mem);
	}

	kfree(file_info);
	return 0;
}

static ssize_t jz_vpu_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	printk("jz-vpu: read is not implemented\n");
	return -1;
}

static ssize_t jz_vpu_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	printk("jz-vpu: write is not implemented\n");
	return -1;
}

static long jz_vpu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct file_info *file_info = filp->private_data;
	spinlock_t ioctl_lock = file_info->ioctl_lock;
	unsigned long flags;

	spin_lock_irqsave(&ioctl_lock, flags);
	switch (cmd) {
	case JZ_VPU_IOCTL_WAIT_COMPLETE:
		dbg_jz_vpu("jz-vpu[%d:%d] ioctl:TCSM_TOCTL_WAIT_COMPLETE\n", current->tgid, current->pid);
                spin_unlock_irqrestore(&ioctl_lock, flags);
                ret = wait_for_completion_interruptible_timeout(&jz_vpu_comp,msecs_to_jiffies(arg));
                spin_lock_irqsave(&ioctl_lock, flags);
		break;
	case JZ_VPU_IOCTL_ALLOC: {
		struct jz_vpu_alloc data;
		copy_from_user(&data, (void*)arg, sizeof(struct jz_vpu_alloc));
                ret = jz_vpu_alloc_phys(file_info, data.size, &data.physical);
		copy_to_user((void*)arg, &data, sizeof(struct jz_vpu_alloc));
		} break;
	case JZ_VPU_IOCTL_FREE:
		jz_vpu_free_phys(file_info, arg);
		break;
	default:
		printk(KERN_ERR "%s:cmd(0x%x) error !!!",__func__,cmd);
		ret = -1;
	}
	spin_unlock_irqrestore(&ioctl_lock, flags);
	return ret;
}

static int jz_vpu_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_flags |= VM_IO;
	/* XXX only set memory to non-cacheable and ioremap when mapping IO, not phys memory */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */
	if (io_remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff, vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static struct miscdevice jz_vpu_dev = {
	MISC_DYNAMIC_MINOR,
	"jz-vpu",
	&jz_vpu_fops
};


/*
 * Module init and exit
 */

static irqreturn_t vpu_interrupt(int irq, void *dev)
{
	CLRREG32(AUX_MIRQP, 0x1);
	complete(&jz_vpu_comp);
	return IRQ_HANDLED;
}

static int __init jz_vpu_init(void)
{
	int ret;

	ret = misc_register(&jz_vpu_dev);
	if (ret < 0) {
		return ret;
	}

	init_completion(&jz_vpu_comp);
	request_irq(IRQ_VPU,vpu_interrupt,IRQF_DISABLED,"jz-vpu",NULL);
	disable_irq_nosync(IRQ_VPU);

	printk("Virtual Driver of JZ VPU registered\n");
	return 0;
}

static void __exit jz_vpu_exit(void)
{
	misc_deregister(&jz_vpu_dev);
	free_irq(IRQ_VPU,NULL);
}

module_init(jz_vpu_init);
module_exit(jz_vpu_exit);

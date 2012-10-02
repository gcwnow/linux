/*
 * linux/arch/mips/jz4770/proc.c
 *
 * /proc/jz/ procfs for jz4770 on-chip modules.
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/page-flags.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>

#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770ddrc.h>
#include <asm/mach-jz4770/jz4770gpio.h>


//#define DEBUG 1
#undef DEBUG

extern void jz4770_fpu_init(unsigned int round);

struct proc_dir_entry *proc_jz_root;


/*
 * DDRC Modules
 */
static int ddrc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf (page+len, "REG_DDRC_ST      0x%08x\n", REG_DDRC_ST);
	len += sprintf (page+len, "REG_DDRC_CFG     0x%08x\n", REG_DDRC_CFG);
	len += sprintf (page+len, "REG_DDRC_CTRL    0x%08x\n", REG_DDRC_CTRL);
	len += sprintf (page+len, "REG_DDRC_LMR     0x%08x\n", REG_DDRC_LMR);
	len += sprintf (page+len, "REG_DDRC_TIMING1 0x%08x\n", REG_DDRC_TIMING1);
	len += sprintf (page+len, "REG_DDRC_TIMING2 0x%08x\n", REG_DDRC_TIMING2);
	len += sprintf (page+len, "REG_DDRC_REFCNT  0x%08x\n", REG_DDRC_REFCNT);
	len += sprintf (page+len, "REG_DDRC_DQS     0x%08x\n", REG_DDRC_DQS);
	len += sprintf (page+len, "REG_DDRC_DQS_ADJ 0x%08x\n", REG_DDRC_DQS_ADJ);
	len += sprintf (page+len, "REG_DDRC_MMAP0   0x%08x\n", REG_DDRC_MMAP0);
	len += sprintf (page+len, "REG_DDRC_MMAP1   0x%08x\n", REG_DDRC_MMAP1);

	return len;
}

/*
 * Power Manager Module
 */
static int pmc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
//	unsigned long clkgr = REG_CPM_CLKGR;
#if 0
	unsigned long lcr = REG_CPM_LCR;
	len += sprintf (page+len, "Low Power Mode : %s\n",
			((lcr & CPM_LCR_LPM_MASK) == (CPM_LCR_LPM_IDLE)) ?
			"IDLE" : (((lcr & CPM_LCR_LPM_MASK) == (CPM_LCR_LPM_SLEEP)) ?
				  "SLEEP" : "HIBERNATE"));
	len += sprintf (page+len, "Doze Mode      : %s\n",
			(lcr & CPM_LCR_DOZE_ON) ? "on" : "off");
	if (lcr & CPM_LCR_DOZE_ON)
		len += sprintf (page+len, "     duty      : %d\n", (int)((lcr & CPM_LCR_DOZE_DUTY_MASK) >> CPM_LCR_DOZE_DUTY_BIT));
#endif
/*
	len += sprintf (page+len, "AUX_CPU        : %s\n",
			(clkgr & CPM_CLKGR_AUX_CPU) ? "stopped" : "running");
	len += sprintf (page+len, "AHB1           : %s\n",
			(clkgr & CPM_CLKGR_AHB1) ? "stopped" : "running");
	len += sprintf (page+len, "IDCT           : %s\n",
			(clkgr & CPM_CLKGR_IDCT) ? "stopped" : "running");
	len += sprintf (page+len, "DB             : %s\n",
			(clkgr & CPM_CLKGR_DB) ? "stopped" : "running");
	len += sprintf (page+len, "ME             : %s\n",
			(clkgr & CPM_CLKGR_ME) ? "stopped" : "running");
	len += sprintf (page+len, "MC             : %s\n",
			(clkgr & CPM_CLKGR_MC) ? "stopped" : "running");
	len += sprintf (page+len, "TVE            : %s\n",
			(clkgr & CPM_CLKGR_TVE) ? "stopped" : "running");
	len += sprintf (page+len, "TSSI           : %s\n",
			(clkgr & CPM_CLKGR_TSSI) ? "stopped" : "running");
	len += sprintf (page+len, "IPU            : %s\n",
			(clkgr & CPM_CLKGR_IPU) ? "stopped" : "running");
	len += sprintf (page+len, "DMAC           : %s\n",
			(clkgr & CPM_CLKGR_DMAC) ? "stopped" : "running");
	len += sprintf (page+len, "LCD            : %s\n",
			(clkgr & CPM_CLKGR_LCD) ? "stopped" : "running");
	len += sprintf (page+len, "CIM            : %s\n",
			(clkgr & CPM_CLKGR_CIM) ? "stopped" : "running");
	len += sprintf (page+len, "SADC           : %s\n",
			(clkgr & CPM_CLKGR_SADC) ? "stopped" : "running");
	len += sprintf (page+len, "MSC0           : %s\n",
			(clkgr & CPM_CLKGR_MSC0) ? "stopped" : "running");
	len += sprintf (page+len, "MSC1           : %s\n",
			(clkgr & CPM_CLKGR_MSC1) ? "stopped" : "running");
	len += sprintf (page+len, "SSI           : %s\n",
			(clkgr & CPM_CLKGR_SSI) ? "stopped" : "running");
	len += sprintf (page+len, "I2C            : %s\n",
			(clkgr & CPM_CLKGR_I2C) ? "stopped" : "running");
	len += sprintf (page+len, "RTC            : %s\n",
			(clkgr & CPM_CLKGR_RTC) ? "stopped" : "running");
	len += sprintf (page+len, "TCU            : %s\n",
			(clkgr & CPM_CLKGR_TCU) ? "stopped" : "running");
	len += sprintf (page+len, "UART1          : %s\n",
			(clkgr & CPM_CLKGR_UART1) ? "stopped" : "running");
	len += sprintf (page+len, "UART0          : %s\n",
			(clkgr & CPM_CLKGR_UART0) ? "stopped" : "running");
*/
	return len;
}

static int pmc_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG_CPM_CLKGR = simple_strtoul(buffer, 0, 16);
	return count;
}

/*
 * Clock Generation Module
 */
#define TO_MHZ(x) (x/1000000),(x%1000000)/10000
#define TO_KHZ(x) (x/1000),(x%1000)/10

static int cgm_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
#if 0
	unsigned int cppcr = REG_CPM_CPPCR;  /* PLL Control Register */
	unsigned int cpccr = REG_CPM_CPCCR;  /* Clock Control Register */
	unsigned int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned int od[4] = {1, 2, 2, 4};

	len += sprintf (page+len, "CPPCR          : 0x%08x\n", cppcr);
	len += sprintf (page+len, "CPCCR          : 0x%08x\n", cpccr);
	len += sprintf (page+len, "PLL            : %s\n",
			(cppcr & CPM_CPPCR_PLLEN) ? "ON" : "OFF");
	len += sprintf (page+len, "m:n:o          : %d:%d:%d\n",
			__cpm_get_pllm() + 2,
			__cpm_get_plln() + 2,
			od[__cpm_get_pllod()]
		);
	len += sprintf (page+len, "C:H:M:P        : %d:%d:%d:%d\n",
			div[__cpm_get_cdiv()],
			div[__cpm_get_hdiv()],
			div[__cpm_get_mdiv()],
			div[__cpm_get_pdiv()]
		);
	len += sprintf (page+len, "PLL Freq        : %3d.%02d MHz\n", TO_MHZ(__cpm_get_pllout()));
	len += sprintf (page+len, "CCLK            : %3d.%02d MHz\n", TO_MHZ(__cpm_get_cclk()));
	len += sprintf (page+len, "HCLK            : %3d.%02d MHz\n", TO_MHZ(__cpm_get_hclk()));
	len += sprintf (page+len, "MCLK            : %3d.%02d MHz\n", TO_MHZ(__cpm_get_mclk()));
	len += sprintf (page+len, "PCLK            : %3d.%02d MHz\n", TO_MHZ(__cpm_get_pclk()));
	len += sprintf (page+len, "H2CLK           : %3d.%02d MHz\n", TO_MHZ(__cpm_get_h2clk()));
	len += sprintf (page+len, "PIXCLK          : %3d.%02d KHz\n", TO_KHZ(__cpm_get_pixclk()));
	len += sprintf (page+len, "I2SCLK          : %3d.%02d MHz\n", TO_MHZ(__cpm_get_i2sclk()));
	len += sprintf (page+len, "USBCLK          : %3d.%02d MHz\n", TO_MHZ(__cpm_get_otgclk()));
	len += sprintf (page+len, "MSC0CLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_mscclk(0)));
	len += sprintf (page+len, "MSC1CLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_mscclk(1)));
	len += sprintf (page+len, "EXTALCLK0       : %3d.%02d MHz\n", TO_MHZ(__cpm_get_extalclk0()));
	len += sprintf (page+len, "EXTALCLK(by CPM): %3d.%02d MHz\n", TO_MHZ(__cpm_get_extalclk()));
	len += sprintf (page+len, "RTCCLK          : %3d.%02d MHz\n", TO_MHZ(__cpm_get_rtcclk()));
#endif
	return len;
}

static int cgm_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG_CPM_CPCCR = simple_strtoul(buffer, 0, 16);
	return count;
}


/* USAGE:
 * echo n  > /proc/jz/ipu 	// n = [1,...,9], alloc mem, 2^n pages.
 * echo FF > /proc/jz/ipu 	// 255, free all buffer
 * echo xxxx > /proc/jz/ipu 	// free buffer which addr is xxxx
 * echo llll > /proc/jz/ipu 	// add_wired_entry(l,l,l,l)
 * echo 0  > /proc/jz/ipu 	// debug, print ipu_buf
 * od -X /proc/jz/ipu 		// read mem addr
 */

typedef struct _ipu_buf {
	unsigned int addr;	/* phys addr */
	unsigned int page_shift;
} ipu_buf_t;

#define IPU_BUF_MAX 		4 	/* 4 buffers */

static struct _ipu_buf ipu_buf[IPU_BUF_MAX];
static int ipu_buf_cnt = 0;
static unsigned char g_asid=0;

extern void local_flush_tlb_all(void);

/* CP0 hazard avoidance. */
#define BARRIER __asm__ __volatile__(".set noreorder\n\t" \
				     "nop; nop; nop; nop; nop; nop;\n\t" \
				     ".set reorder\n\t")
void show_tlb(void)
{
#define ASID_MASK 0xFF

        unsigned long flags;
        unsigned int old_ctx;
	unsigned int entry;
	unsigned int entrylo0, entrylo1, entryhi;
	unsigned int pagemask;

	local_irq_save(flags);

	/* Save old context */
	old_ctx = (read_c0_entryhi() & 0xff);

	printk("TLB content:\n");
	entry = 0;
	while(entry < 32) {
		write_c0_index(entry);
		BARRIER;
		tlb_read();
		BARRIER;
		entryhi = read_c0_entryhi();
		entrylo0 = read_c0_entrylo0();
		entrylo1 = read_c0_entrylo1();
		pagemask = read_c0_pagemask();
		printk("%02d: ASID=%02d%s VA=0x%08x ", entry, entryhi & ASID_MASK, (entrylo0 & entrylo1 & 1) ? "(G)" : "   ", entryhi & ~ASID_MASK);
		printk("PA0=0x%08x C0=%x %s%s%s\n", (entrylo0>>6)<<12, (entrylo0>>3) & 7, (entrylo0 & 4) ? "Dirty " : "", (entrylo0 & 2) ? "Valid " : "Invalid ", (entrylo0 & 1) ? "Global" : "");
		printk("\t\t\t     PA1=0x%08x C1=%x %s%s%s\n", (entrylo1>>6)<<12, (entrylo1>>3) & 7, (entrylo1 & 4) ? "Dirty " : "", (entrylo1 & 2) ? "Valid " : "Invalid ", (entrylo1 & 1) ? "Global" : "");

		printk("\t\tpagemask=0x%08x", pagemask);
		printk("\tentryhi=0x%08x\n", entryhi);
		printk("\t\tentrylo0=0x%08x", entrylo0);
		printk("\tentrylo1=0x%08x\n", entrylo1);

		entry++;
	}
	BARRIER;
	write_c0_entryhi(old_ctx);

	local_irq_restore(flags);
}

static void ipu_add_wired_entry(unsigned long pid,
				unsigned long entrylo0, unsigned long entrylo1,
				unsigned long entryhi, unsigned long pagemask)
{
	unsigned long flags;
	unsigned long wired;
	unsigned long old_pagemask;
	unsigned long old_ctx;
	struct task_struct *g, *p;

	/* We will lock an 4MB page size entry to map the 4MB reserved IPU memory */
	wired = read_c0_wired();
	if (wired) return;

	do_each_thread(g, p) {
		if (p->pid == pid)
			g_asid = p->mm->context.asid[0];
	} while_each_thread(g, p);


	local_irq_save(flags);

	entrylo0 = entrylo0 >> 6;   /* PFN */
	entrylo0 |= 0x6 | (0 << 3); /* Write-through cacheable, dirty, valid */

	/* Save old context and create impossible VPN2 value */
	old_ctx = read_c0_entryhi() & 0xff;
	old_pagemask = read_c0_pagemask();
	wired = read_c0_wired();
	write_c0_wired(wired + 1);
	write_c0_index(wired);
	BARRIER;
	entryhi &= ~0xff;	/* new add, 20070906 */
	entryhi |= g_asid;	/* new add, 20070906 */
//	entryhi |= old_ctx;	/* new add, 20070906 */
	write_c0_pagemask(pagemask);
	write_c0_entryhi(entryhi);
	write_c0_entrylo0(entrylo0);
	write_c0_entrylo1(entrylo1);
	BARRIER;
	tlb_write_indexed();
	BARRIER;

	write_c0_entryhi(old_ctx);
	BARRIER;
	write_c0_pagemask(old_pagemask);
	local_flush_tlb_all();
	local_irq_restore(flags);
#if defined(DEBUG)
	printk("\nold_ctx=%03d\n", old_ctx);

	show_tlb();
#endif
}

static void ipu_del_wired_entry( void )
{
	unsigned long flags;
	unsigned long wired;

	local_irq_save(flags);
	wired = read_c0_wired();
	if ( wired > 0 ) {
		write_c0_wired(wired - 1);
	}
	local_irq_restore(flags);
}

static inline void ipu_buf_get( unsigned int page_shift )
{
	unsigned char * virt_addr;
	int i;
	for ( i=0; i< IPU_BUF_MAX; ++i ) {
		if ( ipu_buf[i].addr == 0 ) {
			break;
		}
	}

	if ( (ipu_buf_cnt = i) == IPU_BUF_MAX ) {
		printk("Error, no free ipu buffer.\n");
		return ;
	}

	virt_addr =  (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);

	if ( virt_addr ) {
		ipu_buf[ipu_buf_cnt].addr = (unsigned int)virt_to_phys((void *)virt_addr);
		ipu_buf[ipu_buf_cnt].page_shift = page_shift;

		for (i = 0; i < (1<<page_shift); i++) {
			SetPageReserved(virt_to_page(virt_addr));
			virt_addr += PAGE_SIZE;
		}
	}
	else {
		printk("get memory Failed.\n");
	}
}

static inline void ipu_buf_free( unsigned int phys_addr )
{
	unsigned char * virt_addr, *addr;
	int cnt, i;

	if ( phys_addr == 0 )
		return ;

	for ( cnt=0; cnt<IPU_BUF_MAX; ++cnt )
		if ( phys_addr == ipu_buf[cnt].addr )
			break;

	if ( cnt == IPU_BUF_MAX ) {	/* addr not in the ipu buffers */
		printk("Invalid addr:0x%08x\n", (unsigned int)phys_addr);
	}

	virt_addr = (unsigned char *)phys_to_virt(ipu_buf[cnt].addr);
	addr = virt_addr;
	for (i = 0; i < (1<<ipu_buf[cnt].page_shift); i++) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
	}

	if ( cnt == 0 )
		ipu_del_wired_entry();

	free_pages((unsigned long )virt_addr, ipu_buf[cnt].page_shift);

	ipu_buf[cnt].addr = 0;
	ipu_buf[cnt].page_shift = 0;
}

static int ipu_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	/* read as binary */
	unsigned int * pint;
	pint = (unsigned int *) (page+len);

	if ( ipu_buf_cnt >= IPU_BUF_MAX ) {	/* failed alloc mem, rturn 0 */
		printk("no free buffer.\n");
		*pint = 0;
	}
	else
		*pint = (unsigned int )ipu_buf[ipu_buf_cnt].addr; /* phys addr */
	len += sizeof(unsigned int);

#if defined(DEBUG)
		show_tlb();
#endif
	return len;

}

static int ipu_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned int val ;
	int cnt,i;
	char buf[12];
	unsigned long pid, entrylo0, entrylo1, entryhi, pagemask;
#if defined(DEBUG)
	printk("ipu write count=%u\n", count);
#endif
	if (count == (8*5+1)) {
		for (i=0;i<12;i++) buf[i]=0;
		strncpy(buf, buffer+8*0, 8);
		pid = simple_strtoul(buf, 0, 16);
		for (i=0;i<12;i++) buf[i]=0;
		strncpy(buf, buffer+8*1, 8);
		entrylo0 = simple_strtoul(buf, 0, 16);
		for (i=0;i<12;i++) buf[i]=0;
		strncpy(buf, buffer+8*2, 8);
		entrylo1 = simple_strtoul(buf, 0, 16);
		for (i=0;i<12;i++) buf[i]=0;
		strncpy(buf, buffer+8*3, 8);
		entryhi = simple_strtoul(buf, 0, 16);
		for (i=0;i<12;i++) buf[i]=0;
		strncpy(buf, buffer+8*4, 8);
		pagemask = simple_strtoul(buf, 0, 16);

#if defined(DEBUG)
		printk("pid=0x%08x, entrylo0=0x%08x, entrylo1=0x%08x, entryhi=0x%08x, pagemask=0x%08x\n",
		       pid, entrylo0, entrylo1, entryhi, pagemask);
#endif
		ipu_add_wired_entry( pid, entrylo0, entrylo1, entryhi, pagemask);
		return 41;
	}
	else if ( count <= 8+1 ) {
		for (i=0;i<12;i++) buf[i]=0;
		strncpy(buf, buffer, 8);
		val = simple_strtoul(buf, 0, 16);
	} else if (count == 44) {
		for (i = 0; i < 12; i++)
			buf[i] = 0;
		strncpy(buf, buffer, 10);
		pid = simple_strtoul(buf, 0, 16);
		for (i = 0; i < 12; i++)
			buf[i] = 0;
		strncpy(buf, buffer + 11, 10);
		entryhi = simple_strtoul(buf, 0, 16);//vaddr
		for (i = 0; i < 12; i++)
			buf[i] = 0;
		strncpy(buf, buffer + 22, 10);
		entrylo0 = simple_strtoul(buf, 0, 16);//paddr
		for (i = 0; i < 12; i++)
			buf[i] = 0;
		strncpy(buf, buffer + 33, 10);
		pagemask = simple_strtoul(buf, 0, 16);
		pagemask = 0x3ff << 13; /* Fixed to 4MB page size */
		ipu_add_wired_entry(pid, entrylo0, 0, entryhi, pagemask);
		return 44;
	} else {
		printk("ipu write count error, count=%d\n.", (unsigned int)count);
		return -1;
	}

	/* val: 1-9, page_shift,  val>= 10: ipu_buf.addr */
	if ( val == 0 ) {	/* debug, print ipu_buf info */
		for ( cnt=0; cnt<IPU_BUF_MAX; ++cnt)
			printk("ipu_buf[%d]: addr=0x%08x, page_shift=%d\n",
			       cnt, ipu_buf[cnt].addr, ipu_buf[cnt].page_shift );
#if defined(DEBUG)
		show_tlb();
#endif
	}
	else if ( 0< val && val < 10 ) {
		ipu_buf_get(val);
	}
	else if ( val == 0xff ) { /* 255: free all ipu_buf */
		for ( cnt=0; cnt<IPU_BUF_MAX; ++cnt ) {
			ipu_buf_free(ipu_buf[cnt].addr);
		}
	}
	else {
		ipu_buf_free(val);
	}

	return count;
}

/*
 * MMC/SD hotplug
 */

#ifndef MSC_HOTPLUG_PIN
#define MSC_HOTPLUG_PIN 90
#endif

static int mmc_read_proc (char *page, char **start, off_t off,
                          int count, int *eof, void *data)
{
        int len = 0;

        if (__gpio_get_pin(MSC_HOTPLUG_PIN))
                len += sprintf (page+len, "REMOVE\n");
        else
                len += sprintf (page+len, "INSERT\n");

        return len;
}

#ifndef CONFIG_ANDROID_PMEM	/* /dev/pmem instead /proc/jz/imem on android platform */

/***********************************************************************
 * IPU memory management (used by mplayer and other apps)
 *
 * We reserved 16MB memory for IPU
 * The memory base address is jz_ipu_framebuf
 */

/* Usage:
 *
 * echo n  > /proc/jz/imem 		// n = [0,...,10], allocate memory, 2^n pages
 * echo xxxxxxxx > /proc/jz/imem	// free buffer which addr is xxxxxxxx
 * echo FF > /proc/jz/ipu 		// FF, free all buffers
 * od -X /proc/jz/imem 			// return the allocated buffer address and the max order of free buffer
 */

//#define DEBUG_IMEM 1

#define IMEM_MAX_ORDER 13		/* max 2^12 * 4096 = 16MB */

static unsigned int jz_imem_base;	/* physical base address of ipu memory */

static unsigned int allocated_phys_addr = 0;

/*
 * Allocated buffer list
 */
typedef struct imem_list {
	unsigned int phys_start;	/* physical start addr */
	unsigned int phys_end;		/* physical end addr */
	struct imem_list *next;
} imem_list_t;

static struct imem_list *imem_list_head = NULL; /* up sorted by phys_start */

#define IMEM1_MAX_ORDER 12              /* max 2^11 * 4096 = 8MB */
static unsigned int jz_imem1_base;      /* physical base address of ipu memory */
static unsigned int allocated_phys_addr1 = 0;
static struct imem_list *imem1_list_head = NULL; /* up sorted by phys_start */


#ifdef DEBUG_IMEM
static void dump_imem_list(void)
{
	struct imem_list *imem;

	printk("*** dump_imem_list 0x%x ***\n", (u32)imem_list_head);
	imem = imem_list_head;
	while (imem) {
		printk("imem=0x%x phys_start=0x%x phys_end=0x%x next=0x%x\n", (u32)imem, imem->phys_start, imem->phys_end, (u32)imem->next);
		imem = imem->next;
	}

        printk("*** dump_imem_list 0x%x ***\n", (u32)imem1_list_head);
        imem = imem1_list_head;
        while (imem) {
                printk("imem=0x%x phys_start=0x%x phys_end=0x%x next=0x%x\n", (u32)imem, imem->phys_start, imem->phys_end, (u32)imem->next);
                imem = imem->next;
        }
}
#endif

/* allocate 2^order pages inside the 16MB memory */
static int imem_alloc(unsigned int order)
{
	int alloc_ok = 0;
	unsigned int start, end;
	unsigned int size = (1 << order) * PAGE_SIZE;
	struct imem_list *imem, *imemn, *imemp;

	allocated_phys_addr = 0;

	start = jz_imem_base;
	end = start + (1 << IMEM_MAX_ORDER) * PAGE_SIZE;

	imem = imem_list_head;
	while (imem) {
		if ((imem->phys_start - start) >= size) {
			/* we got a valid address range */
			alloc_ok = 1;
			break;
		}

		start = imem->phys_end + 1;
		imem = imem->next;
	}

	if (!alloc_ok) {
		if ((end - start) >= size)
			alloc_ok = 1;
	}

	if (alloc_ok) {
		end = start + size - 1;
		allocated_phys_addr = start;

		/* add to imem_list, up sorted by phys_start */
		imemn = kmalloc(sizeof(struct imem_list), GFP_KERNEL);
		if (!imemn) {
			printk("-->%s, kmalloc failed1\n", __FUNCTION__);
			return -ENOMEM;
		}
		imemn->phys_start = start;
		imemn->phys_end = end;
		imemn->next = NULL;

		if (!imem_list_head)
			imem_list_head = imemn;
		else {
			imem = imemp = imem_list_head;
			while (imem) {
				if (start < imem->phys_start) {
					break;
				}

				imemp = imem;
				imem = imem->next;
			}

			if (imem == imem_list_head) {
				imem_list_head = imemn;
				imemn->next = imem;
			}
			else {
				imemn->next = imemp->next;
				imemp->next = imemn;
			}
		}
	}

#ifdef DEBUG_IMEM
	dump_imem_list();
#endif
	return 0;
}

/* allocate 2^order pages inside the 8MB memory */
static int imem1_alloc(unsigned int order)
{
	int alloc_ok = 0;
	unsigned int start, end;
	unsigned int size = (1 << order) * PAGE_SIZE;
	struct imem_list *imem, *imemn, *imemp;

	allocated_phys_addr1 = 0;

	start = jz_imem1_base;
	end = start + (1 << IMEM1_MAX_ORDER) * PAGE_SIZE;

	imem = imem1_list_head;
	while (imem) {
		if ((imem->phys_start - start) >= size) {
			/* we got a valid address range */
			alloc_ok = 1;
			break;
		}

		start = imem->phys_end + 1;
		imem = imem->next;
	}

	if (!alloc_ok) {
		if ((end - start) >= size)
			alloc_ok = 1;
	}

	if (alloc_ok) {
		end = start + size - 1;
		allocated_phys_addr1 = start;

		/* add to imem_list, up sorted by phys_start */
		imemn = kmalloc(sizeof(struct imem_list), GFP_KERNEL);
		if (!imemn) {
			printk("-->%s, kmalloc failed2\n", __FUNCTION__);
			return -ENOMEM;
		}
		imemn->phys_start = start;
		imemn->phys_end = end;
		imemn->next = NULL;

		if (!imem1_list_head)
			imem1_list_head = imemn;
		else {
			imem = imemp = imem1_list_head;
			while (imem) {
				if (start < imem->phys_start) {
					break;
				}

				imemp = imem;
				imem = imem->next;
			}

			if (imem == imem1_list_head) {
				imem1_list_head = imemn;
				imemn->next = imem;
			}
			else {
				imemn->next = imemp->next;
				imemp->next = imemn;
			}
		}
	}

#ifdef DEBUG_IMEM
	dump_imem_list();
#endif
	return 0;
}

static void imem_free(unsigned int phys_addr)
{
	struct imem_list *imem, *imemp;

	imem = imemp = imem_list_head;
	while (imem) {
		if (phys_addr == imem->phys_start) {
			if (imem == imem_list_head) {
				imem_list_head = imem->next;
			}
			else {
				imemp->next = imem->next;
			}

			kfree(imem);
			break;
		}

		imemp = imem;
		imem = imem->next;
	}

        imem = imemp = imem1_list_head;
        while (imem) {
                if (phys_addr == imem->phys_start) {
                        if (imem == imem1_list_head) {
                                imem1_list_head = imem->next;
                        }
                        else {
                                imemp->next = imem->next;
                        }

                        kfree(imem);
                        break;
                }

                imemp = imem;
                imem = imem->next;
        }

#ifdef DEBUG_IMEM
	dump_imem_list();
#endif
}

static void imem_free_all(void)
{
	struct imem_list *imem;

	imem = imem_list_head;
	while (imem) {
		kfree(imem);
		imem = imem->next;
	}

	imem_list_head = NULL;

	allocated_phys_addr = 0;

        imem = imem1_list_head;
        while (imem) {
                kfree(imem);
                imem = imem->next;
        }

        imem1_list_head = NULL;

        allocated_phys_addr1 = 0;

#ifdef DEBUG_IMEM
	dump_imem_list();
#endif
}

/*
 * Return the allocated buffer address and the max order of free buffer
 */
static int imem_read_proc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned int start_addr, end_addr, max_order, max_size;
	struct imem_list *imem;

	unsigned int *tmp = (unsigned int *)(page + len);

	start_addr = jz_imem_base;
	end_addr = start_addr + (1 << IMEM_MAX_ORDER) * PAGE_SIZE;

	if (!imem_list_head)
		max_size = end_addr - start_addr;
	else {
		max_size = 0;
		imem = imem_list_head;
		while (imem) {
			if (max_size < (imem->phys_start - start_addr))
				max_size = imem->phys_start - start_addr;

			start_addr = imem->phys_end + 1;
			imem = imem->next;
		}

		if (max_size < (end_addr - start_addr))
			max_size = end_addr - start_addr;
	}

	if (max_size > 0) {
		max_order = get_order(max_size);
		if (((1 << max_order) * PAGE_SIZE) > max_size)
		    max_order--;
	}
	else {
		max_order = 0xffffffff;	/* No any free buffer */
	}

	*tmp++ = allocated_phys_addr;	/* address allocated by 'echo n > /proc/jz/imem' */
	*tmp = max_order;		/* max order of current free buffers */

	len += 2 * sizeof(unsigned int);

	return len;
}

static int imem_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned int val;

	val = simple_strtoul(buffer, 0, 16);

	if (val == 0xff) {
		/* free all memory */
		imem_free_all();
	}
	else if ((val >= 0) && (val <= IMEM_MAX_ORDER)) {
		/* allocate 2^val pages */
		imem_alloc(val);
	}
	else {
		/* free buffer which phys_addr is val */
		imem_free(val);
	}

	return count;
}

/*
 * Return the allocated buffer address and the max order of free buffer
 */
static int imem1_read_proc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned int start_addr, end_addr, max_order, max_size;
	struct imem_list *imem;

	unsigned int *tmp = (unsigned int *)(page + len);

	start_addr = jz_imem1_base;
	end_addr = start_addr + (1 << IMEM1_MAX_ORDER) * PAGE_SIZE;

	if (!imem1_list_head)
		max_size = end_addr - start_addr;
	else {
		max_size = 0;
		imem = imem1_list_head;
		while (imem) {
			if (max_size < (imem->phys_start - start_addr))
				max_size = imem->phys_start - start_addr;

			start_addr = imem->phys_end + 1;
			imem = imem->next;
		}

		if (max_size < (end_addr - start_addr))
			max_size = end_addr - start_addr;
	}

	if (max_size > 0) {
		max_order = get_order(max_size);
		if (((1 << max_order) * PAGE_SIZE) > max_size)
		    max_order--;
	}
	else {
		max_order = 0xffffffff;	/* No any free buffer */
	}

	*tmp++ = allocated_phys_addr1;	/* address allocated by 'echo n > /proc/jz/imem' */
	*tmp = max_order;		/* max order of current free buffers */

	len += 2 * sizeof(unsigned int);

	return len;
}

static int imem1_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned int val;

	val = simple_strtoul(buffer, 0, 16);

	if (val == 0xff) {
		/* free all memory */
		imem_free_all();
		ipu_del_wired_entry();
	} else if ((val >= 0) && (val <= IMEM1_MAX_ORDER)) {
		/* allocate 2^val pages */
		imem1_alloc(val);
	} else {
		/* free buffer which phys_addr is val */
		imem_free(val);
	}

	return count;
}

#endif	/* #ifndef CONFIG_ANDROID_PMEM */

static int fpu_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	printk("user set rounding mode to %x \n",(unsigned int)buffer);

	if ((unsigned int)buffer > 4) {
		printk("roundind mode error!\n");
	}

	jz4770_fpu_init((unsigned int)buffer);
	return count;
}

/*
 * /proc/jz/xxx entry
 *
 */
static int __init jz_proc_init(void)
{
	struct proc_dir_entry *res;
#ifndef CONFIG_ANDROID_PMEM
	unsigned int virt_addr, i;
#endif

	proc_jz_root = proc_mkdir("jz", 0);

	/* Power Management Controller */
	res = create_proc_entry("pmc", 0644, proc_jz_root);
	if (res) {
		res->read_proc = pmc_read_proc;
		res->write_proc = pmc_write_proc;
		res->data = NULL;
	}

	/* Clock Generation Module */
	res = create_proc_entry("cgm", 0644, proc_jz_root);
	if (res) {
		res->read_proc = cgm_read_proc;
		res->write_proc = cgm_write_proc;
		res->data = NULL;
	}

	/* Image process unit */
	res = create_proc_entry("ipu", 0644, proc_jz_root);
	if (res) {
		res->read_proc = ipu_read_proc;
		res->write_proc = ipu_write_proc;
		res->data = NULL;
	}

	/* mmc hotplug */
	res = create_proc_entry("mmc", 0644, proc_jz_root);
	if (res) {
		res->read_proc = mmc_read_proc;
		res->write_proc = NULL;
		res->data = NULL;
	}

        /* DDR Controller */
	res = create_proc_entry("ddrc", 0644, proc_jz_root);
	if (res) {
		res->read_proc = ddrc_read_proc;
		res->write_proc = NULL;
		res->data = NULL;
	}

#ifndef CONFIG_ANDROID_PMEM
	/*
	 * Reserve a 16MB memory for IPU on JZ4770.
	 */
	jz_imem_base = (unsigned int)__get_free_pages(GFP_KERNEL, IMEM_MAX_ORDER);
	if (jz_imem_base) {
		/* imem (IPU memory management) */
		res = create_proc_entry("imem", 0644, proc_jz_root);
		if (res) {
			res->read_proc = imem_read_proc;
			res->write_proc = imem_write_proc;
			res->data = NULL;
		}

		/* Set page reserved */
		virt_addr = jz_imem_base;
		for (i = 0; i < (1 << IMEM_MAX_ORDER); i++) {
			SetPageReserved(virt_to_page((void *)virt_addr));
			virt_addr += PAGE_SIZE;
		}

		/* Convert to physical address */
		jz_imem_base = virt_to_phys((void *)jz_imem_base);

		printk("Total %dMB memory at 0x%x was reserved for IPU\n",
		       (unsigned int)((1 << IMEM_MAX_ORDER) * PAGE_SIZE)/1000000, jz_imem_base);
	}
        else
           printk("NOT enough memory for imem\n");

        jz_imem1_base = (unsigned int)__get_free_pages(GFP_KERNEL, IMEM1_MAX_ORDER);
        if (jz_imem1_base) {
                /* imem (IPU memory management) */
                res = create_proc_entry("imem1", 0644, proc_jz_root);
                if (res) {
                        res->read_proc = imem1_read_proc;
                        res->write_proc = imem1_write_proc;
                        res->data = NULL;
                }

                /* Set page reserved */
                virt_addr = jz_imem1_base;
                for (i = 0; i < (1 << IMEM1_MAX_ORDER); i++) {
                        SetPageReserved(virt_to_page((void *)virt_addr));
                        virt_addr += PAGE_SIZE;
                }

                /* Convert to physical address */
                jz_imem1_base = virt_to_phys((void *)jz_imem1_base);

                printk("Total %dMB memory1 at 0x%x was reserved for IPU\n",
                       (unsigned int)((1 << IMEM1_MAX_ORDER) * PAGE_SIZE)/1000000, jz_imem1_base);
        }
        else
           printk("NOT enough memory for imem1\n");

#endif	/* #ifdef CONFIG_ANDROID_PMEM */

	/* fpu */
	res = create_proc_entry("fpu", 0644, proc_jz_root);
	if (res) {
		res->read_proc = NULL;
		res->write_proc = fpu_write_proc;
		res->data = NULL;
	}

	return 0;
}

__initcall(jz_proc_init);

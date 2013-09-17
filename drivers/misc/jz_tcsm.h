
#ifndef __JZ_TCSM_H
#define __JZ_TCSM_H

#define TCSM_MINOR              242

#define TCSM_TOCTL_SET_MMAP 		(0x99 + 0x1)
#define TCSM_TOCTL_WAIT_COMPLETE 	(0x99 + 0x2)
#define TCSM_TOCTL_PREPARE_DIR		(0x99 + 0x10)
#define TCSM_TOCTL_PREPARE_BLOCK	(0x99 + 0x11)
#define TCSM_TOCTL_FLUSH_WORK		(0x99 + 0x12)

#if 1
#define dbg_tcsm(x...) printk(x)
#else
#define dbg_tcsm(x...)
#endif

struct tcsm_mmap {
	unsigned int start;
	unsigned int len;
};

enum tcsm_file_mode {
	UNOPENED = 0,
	R_ONLY,
	W_ONLY,
	R_W
};

enum tcsm_file_cmd {
	CAN_OPEN = 0,
	BLOCK,
	RETURN_NOW
};

struct tcsm_sem {
	enum tcsm_file_mode tcsm_file_mode_now;
	enum tcsm_file_mode tcsm_file_mode_pre;
	struct semaphore sem;
	pid_t owner_pid;
	unsigned char is_on;
};

struct file_info {
	unsigned char is_on;
	spinlock_t ioctl_lock;
};
#endif

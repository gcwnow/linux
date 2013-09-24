
#ifndef __JZ_VPU_H
#define __JZ_VPU_H

#define JZ_VPU_MINOR              242

#define TCSM_TOCTL_WAIT_COMPLETE	(0x99 + 0x2)

#if 1
#define dbg_jz_vpu(x...) printk(x)
#else
#define dbg_jz_vpu(x...)
#endif

enum jz_vpu_file_mode {
	UNOPENED = 0,
	R_ONLY,
	W_ONLY,
	R_W
};

enum jz_vpu_file_cmd {
	CAN_OPEN = 0,
	BLOCK,
	RETURN_NOW
};

struct jz_vpu_sem {
	enum jz_vpu_file_mode jz_vpu_file_mode_now;
	enum jz_vpu_file_mode jz_vpu_file_mode_pre;
	struct semaphore sem;
	pid_t owner_pid;
};

struct file_info {
	spinlock_t ioctl_lock;
};
#endif

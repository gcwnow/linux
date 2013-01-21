#
# Vivante GC860 for Ingenic JZ4770 SOC.
#
obj-$(CONFIG_VIVANTE_GPU_GC860) += gc860.o

gc860-objs := hal/os/linux/kernel/gc_hal_kernel_linux.o		\
	      hal/os/linux/kernel/gc_hal_kernel_debug.o		\
	      hal/os/linux/kernel/gc_hal_kernel_device.o	\
	      hal/os/linux/kernel/gc_hal_kernel_os.o		\
	      hal/os/linux/kernel/gc_hal_kernel_driver.o	\
	      hal/kernel/gc_hal_kernel_mmu.o			\
	      hal/kernel/gc_hal_kernel_heap.o			\
	      hal/kernel/gc_hal_kernel_video_memory.o		\
	      hal/kernel/gc_hal_kernel_command.o		\
	      hal/kernel/gc_hal_kernel_event.o			\
	      hal/kernel/gc_hal_kernel.o			\
	      arch/XAQ2/hal/kernel/gc_hal_kernel_hardware.o

EXTRA_CFLAGS += -DLINUX -DDRIVER -DENUM_WORKAROUND=0 -DDBG=0 -DNO_DMA_COHERENT -DENABLE_ARM_L2_CACHE=0 -DgcdNO_POWER_MANAGEMENT=0 -DUSE_PLATFORM_DRIVER=1 -DVIVANTE_PROFILER=0 -DANDROID=1 -DENABLE_GPU_CLOCK_BY_DRIVER=0 -DUSE_NEW_LINUX_SIGNAL=0 -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=0 -DgcdkREPORT_VIDMEM_USAGE=0 -I drivers/gpu/vivante/GC860/hal/inc -I drivers/gpu/vivante/GC860/hal/kernel/inc -I drivers/gpu/vivante/GC860/hal/kernel -I drivers/gpu/vivante/GC860/arch/XAQ2/hal/kernel -I drivers/gpu/vivante/GC860/hal/user -DgcdENABLE_BANK_ALIGNMENT=0 -DFIXED_MMAP_AS_CACHEABLE=0 -Wframe-larger-than=1032 -Werror

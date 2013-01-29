#
# Vivante WK860 for Ingenic JZ4770 SOC.
#
obj-$(CONFIG_VIVANTE_GPU_GC860) += galcore.o

galcore-objs := hal/kernel/gc_hal_kernel.o			\
		hal/kernel/gc_hal_kernel_command.o		\
		hal/kernel/gc_hal_kernel_db.o			\
		hal/kernel/gc_hal_kernel_debug.o		\
		hal/kernel/gc_hal_kernel_event.o		\
		hal/kernel/gc_hal_kernel_heap.o			\
		hal/kernel/gc_hal_kernel_mmu.o			\
		hal/kernel/gc_hal_kernel_video_memory.o		\
		hal/os/linux/kernel/gc_hal_kernel_device.o	\
		hal/os/linux/kernel/gc_hal_kernel_driver.o	\
		hal/os/linux/kernel/gc_hal_kernel_linux.o	\
		hal/os/linux/kernel/gc_hal_kernel_math.o	\
		hal/os/linux/kernel/gc_hal_kernel_os.o		\
		arch/XAQ2/hal/kernel/gc_hal_kernel_context.o	\
		arch/XAQ2/hal/kernel/gc_hal_kernel_hardware.o
   

EXTRA_CFLAGS += -DLINUX -DDRIVER -DENUM_WORKAROUND=0 -DDBG=1 -DDEBUG=1 -D_DEBUG=1 -DNO_DMA_COHERENT -DENABLE_ARM_L2_CACHE=0 -DgcdNO_POWER_MANAGEMENT=0 -DUSE_PLATFORM_DRIVER=0 -DVIVANTE_PROFILER=0 -DANDROID=1 -DENABLE_GPU_CLOCK_BY_DRIVER=0 -DUSE_NEW_LINUX_SIGNAL=0 -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=1 -DgcdkREPORT_VIDMEM_USAGE=0 -I drivers/gpu/vivante/WK860/hal/inc -I drivers/gpu/vivante/WK860/hal/kernel/inc -I drivers/gpu/vivante/WK860/hal/kernel -I drivers/gpu/vivante/WK860/arch/XAQ2/hal/kernel -I drivers/gpu/vivante/WK860/hal/os/linux/kernel -DgcdENABLE_BANK_ALIGNMENT=0 -DFIXED_MMAP_AS_CACHEABLE=0 -Wframe-larger-than=1032 -Werror -DgcdENABLE_VG=0 -DgcdCACHE_FUNCTION_UNIMPLEMENTED=0 -DgcdPOWER_CONTROL_DELAY=1 -DgcdMMU_SIZE=262144

#
# Reverse engineered Vivante Driver w/JZ4770 support.
#
obj-$(CONFIG_GPU_VIVANTE_V2) += etnaviv.o
ccflags-$(CONFIG_GPU_VIVANTE_V2_DEBUG) += -DDEBUG -D_DEBUG -DDBG=1
ccflags-$(CONFIG_GPU_VIVANTE_V2_PROFILER) += -DVIVANTE_PROFILER=1

etnaviv-objs := hal/os/linux/kernel/gc_hal_kernel_linux.o		\
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

#
# Do a kernel flush_cache_all instead of user flush_cache_range
#
EXTRA_CFLAGS += -DFLUSH_CACHE_ALL_IN_KERNEL=1

#
# Make mmap() be cacheable and implement gckOS_CacheFlush()
#
#EXTRA_CFLAGS += -DFIXED_MMAP_AS_CACHEABLE=1
EXTRA_CFLAGS += -DFIXED_MMAP_AS_CACHEABLE=0

#
# Set to 1 if platform needs copy_from_user to access user space from kernel
#
EXTRA_CFLAGS += -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=0

#
# Define if dma_mmap_coherent is not needed
#
EXTRA_CFLAGS += -DNO_DMA_COHERENT

EXTRA_CFLAGS += -DLINUX -DDRIVER -DENABLE_ARM_L2_CACHE=0 -DgcdNO_POWER_MANAGEMENT=0 -DUSE_PLATFORM_DRIVER=1 \
		-DANDROID=0 -DENABLE_GPU_CLOCK_BY_DRIVER=0 -DUSE_NEW_LINUX_SIGNAL=0  -DgcdkREPORT_VIDMEM_USAGE=0 \
		-I drivers/gpu/vivante/ETNAVIV/hal/inc -I drivers/gpu/vivante/ETNAVIV/hal/inc \
		-I drivers/gpu/vivante/ETNAVIV/hal/kernel -I drivers/gpu/vivante/ETNAVIV/arch/XAQ2/hal/kernel \
		-I drivers/gpu/vivante/ETNAVIV/hal/user \
		-Wframe-larger-than=1032 -Werror -DgcdNULL_DRIVER=0

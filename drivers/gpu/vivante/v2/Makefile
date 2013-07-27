#
# Open Vivante driver Makefile.
#
ccflags-$(CONFIG_GPU_VIVANTE_V2_DEBUG) += -DDEBUG -D_DEBUG -DDBG=1
ccflags-$(CONFIG_GPU_VIVANTE_V2_PROFILER) += -DVIVANTE_PROFILER=1

#
# Do a kernel flush_cache_all instead of user flush_cache_range
#
ccflags-y += -DFLUSH_CACHE_ALL_IN_KERNEL=1

#
# Make mmap() be cacheable and implement gckOS_CacheFlush()
#
#ccflags-y += -DFIXED_MMAP_AS_CACHEABLE=1
ccflags-y += -DFIXED_MMAP_AS_CACHEABLE=0

#
# Set to 1 if platform needs copy_from_user to access user space from kernel
#
EXTRA_CFLAGS += -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=0

#
# Define if dma_mmap_coherent is not needed
#
ccflags-y += -DNO_DMA_COHERENT

ccflags-y += -DLINUX
ccflags-y += -DDRIVER
ccflags-y += -DENABLE_ARM_L2_CACHE=0
ccflags-y += -DUSE_PLATFORM_DRIVER=1
ccflags-y += -DANDROID=0
ccflags-y += -DENABLE_GPU_CLOCK_BY_DRIVER=0
ccflags-y += -DUSE_NEW_LINUX_SIGNAL=0
ccflags-y += -DgcdkREPORT_VIDMEM_USAGE=0
ccflags-y += -DgcdNO_POWER_MANAGEMENT=1
ccflags-y += -DgcdNULL_DRIVER=0

# Avoid warning.
ccflags-y += -Wframe-larger-than=1032

#
# Include paths.
#
ccflags-y += \
	-I$(obj)/arch/XAQ2/hal/kernel \
	-I$(obj)/hal/inc \
	-I$(obj)/hal/kernel \
	-I$(obj)/hal/user

obj-$(CONFIG_GPU_VIVANTE_V2) += galcore_v2.o

galcore_v2-objs += \
	arch/XAQ2/hal/kernel/gc_hal_kernel_hardware.o	\
	hal/kernel/gc_hal_kernel.o			\
	hal/kernel/gc_hal_kernel_command.o		\
	hal/kernel/gc_hal_kernel_event.o		\
	hal/kernel/gc_hal_kernel_heap.o			\
	hal/kernel/gc_hal_kernel_mmu.o			\
	hal/kernel/gc_hal_kernel_video_memory.o		\
	hal/os/linux/kernel/gc_hal_kernel_debug.o	\
	hal/os/linux/kernel/gc_hal_kernel_device.o	\
	hal/os/linux/kernel/gc_hal_kernel_driver.o	\
	hal/os/linux/kernel/gc_hal_kernel_linux.o	\
	hal/os/linux/kernel/gc_hal_kernel_os.o

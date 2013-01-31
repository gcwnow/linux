#
# Open Vivante driver Makefile.
#
ccflags-$(CONFIG_GPU_VIVANTE_NEWSIGNAL) += -DUSE_NEW_LINUX_SIGNAL=1
ccflags-$(CONFIG_GPU_VIVANTE_PROFILER) += -DVIVANTE_PROFILER=1
ccflags-$(CONFIG_GPU_VIVANTE_ANDROID) += -DANDROID=1
ccflags-$(CONFIG_GPU_VIVANTE_DEBUG) += -DDEBUG -D_DEBUG -DDBG=1
ccflags-$(CONFIG_GPU_VIVANTE_NO_DMA_COHERENT) += -DNO_DMA_COHERENT
ccflags-$(CONFIG_SMP) += -DgcdSMP=1

#ccflags-y += -DUSE_PLATFORM_DRIVER=1
ccflags-y += -DENUM_WORKAROUND=0
ccflags-y += -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=1
ccflags-y += -DgcdPAGED_MEMORY_CACHEABLE=0
ccflags-y += -DgcdNONPAGED_MEMORY_CACHEABLE=0
ccflags-y += -DgcdNONPAGED_MEMORY_BUFFERABLE=1
ccflags-y += -DgcdCACHE_FUNCTION_UNIMPLEMENTED=0
ccflags-y += -DgcdENABLE_VG=0
ccflags-y += -DgcdSUPPORT_SWAP_RECTANGLE=0
ccflags-y += -DgcdENABLE_OUTER_CACHE_PATCH=0
ccflags-y += -DgcdUSE_OPENCL=0
ccflags-y += -Werror

#
# Vivante GC860 for Ingenic JZ4770 SOC.
#
ifneq ($(CONFIG_GPU_VIVANTE_GC860), )
ccflags-y += \
	-DgcdMMU_SIZE=262144 \
	-DgcdPOWER_CONTROL_DELAY=1 \
	-DgcdPOWER_MANAGEMENT=0 \
	-DUSE_PLATFORM_DRIVER=0 \
	-DNO_DMA_COHERENT
endif

#
# Include paths.
#
ccflags-y += \
	-I$(obj)/hal/kernel \
	-I$(obj)/hal/kernel/inc \
	-I$(obj)/hal/os/linux/kernel \
	-I$(obj)/arch/XAQ2/hal/kernel

obj-$(CONFIG_GPU_VIVANTE_V4) += galcore.o

galcore-objs += \
	hal/kernel/gc_hal_kernel.o \
	hal/kernel/gc_hal_kernel_command.o \
	hal/kernel/gc_hal_kernel_db.o \
	hal/kernel/gc_hal_kernel_debug.o \
	hal/kernel/gc_hal_kernel_event.o \
	hal/kernel/gc_hal_kernel_heap.o \
	hal/kernel/gc_hal_kernel_mmu.o \
	hal/kernel/gc_hal_kernel_video_memory.o \
	hal/os/linux/kernel/gc_hal_kernel_device.o \
	hal/os/linux/kernel/gc_hal_kernel_driver.o \
	hal/os/linux/kernel/gc_hal_kernel_linux.o \
	hal/os/linux/kernel/gc_hal_kernel_math.o \
	hal/os/linux/kernel/gc_hal_kernel_os.o \
	arch/XAQ2/hal/kernel/gc_hal_kernel_context.o \
	arch/XAQ2/hal/kernel/gc_hal_kernel_hardware.o

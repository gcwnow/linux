
OS_KERNEL_DIR   := hal/os/linux/kernel
ARCH_KERNEL_DIR := arch/XAQ2/hal/kernel
HAL_KERNEL_DIR  := hal/kernel

VIV_DRIVER_PATH := drivers/gpu/vivante/VIVANTE_GAL_Unified_Src_drv_2.5.3.2.5.b7.noGPL

# GC860 galcore config # -Werror 
EXTRA_CFLAGS += -DLINUX -DDRIVER -DENUM_WORKAROUND=0 -DDBG=0 -DNO_DMA_COHERENT -DENABLE_ARM_L2_CACHE=0 -DgcdNO_POWER_MANAGEMENT=0 -DUSE_PLATFORM_DRIVER=1 -DVIVANTE_PROFILER=0 -DANDROID=1 -DENABLE_GPU_CLOCK_BY_DRIVER=0 -DUSE_NEW_LINUX_SIGNAL=0 -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=0 -DgcdkREPORT_VIDMEM_USAGE=0 -I $(VIV_DRIVER_PATH)/hal/inc -I $(VIV_DRIVER_PATH)/hal/kernel/inc -I $(VIV_DRIVER_PATH)/hal/kernel -I $(VIV_DRIVER_PATH)/arch/XAQ2/hal/kernel -I $(VIV_DRIVER_PATH)/hal/user/

obj-y += $(OS_KERNEL_DIR)/gc_hal_kernel_linux.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_debug.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_device.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_os.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_driver.o


obj-y += $(HAL_KERNEL_DIR)/gc_hal_kernel_mmu.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_heap.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_video_memory.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_command.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_event.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel.o

obj-y += $(ARCH_KERNEL_DIR)/gc_hal_kernel_hardware.o


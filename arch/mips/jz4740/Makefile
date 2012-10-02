#
# Makefile for the Ingenic JZ4740.
#

# Object file lists.

obj-y += prom.o setup.o

obj-$(CONFIG_MACH_JZ4740) += gpio.o
obj-$(CONFIG_MACH_JZ4770) += jz4770-pm.o
obj-$(CONFIG_MACH_JZ4740)$(CONFIG_MACH_JZ4780) += \
	time.o timer.o platform.o reset.o

CFLAGS_setup.o = -I$(src)/../../../scripts/dtc/libfdt

# board specific support

obj-$(CONFIG_JZ4740_QI_LB60)	+= board-qi_lb60.o
obj-$(CONFIG_JZ4770_GCW0)		+= board-gcw0.o

# PM support

obj-$(if CONFIG_MACH_JZ4770,,$(CONFIG_PM)) += pm.o

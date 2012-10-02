#
# Makefile for the Ingenic JZ4770.
#

# Object file lists.

obj-y += prom.o reset.o setup.o platform.o fpu.o
obj-y += irq.o gpio.o dma.o time.o cpm.o clock.o

obj-$(CONFIG_JZ4770_PROC_JZ)	+= proc.o

# board specific support
obj-$(CONFIG_JZ4770_GCW0)	+= board-gcw0.o

# PM support

obj-$(CONFIG_PM)		+= pm.o sleep.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)	+= cpufreq.o

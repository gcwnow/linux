#
# Makefile for the Ingenic JZ4770.
#

# Object file lists.

obj-y += prom.o reset.o setup.o platform.o
obj-y += gpio.o dma.o time.o clock.o

# board specific support
obj-$(CONFIG_JZ4770_GCW0)	+= board-gcw0.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)	+= cpufreq.o

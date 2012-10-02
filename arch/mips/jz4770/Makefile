#
# Makefile for the Ingenic JZ4770.
#

# Object file lists.

obj-y += prom.o reset.o setup.o platform.o
obj-y += irq.o gpio.o dma.o time.o clock.o

# board specific support

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)	+= cpufreq.o

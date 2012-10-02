#
# Makefile for the Ingenic JZ4770.
#

# Object file lists.

obj-y += prom.o setup.o platform.o

# board specific support

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)	+= cpufreq.o

# Suspend/resume support
obj-$(CONFIG_PM_SLEEP)		+= pm.o

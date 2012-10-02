#
# Makefile for the Ingenic JZ4770.
#

# Object file lists.

obj-y += prom.o setup.o

# board specific support
obj-$(CONFIG_JZ4770_GCW0)	+= board-gcw0.o

# Suspend/resume support
obj-$(CONFIG_PM_SLEEP)		+= pm.o

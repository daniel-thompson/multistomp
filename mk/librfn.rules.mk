#
# librfn.rules.mk
# 
# This file of the multistomp firmware.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#

LIBRFN_DIR = ../../librfn

OBJS += \
	fibre.o \
	fibre_default.o \
	list.o \
	messageq.o \
	time_libopencm3.o \
	util.o

vpath %.c $(LIBRFN_DIR)/lib
CPPFLAGS += -DNDEBUG
CPPFLAGS += -I$(LIBRFN_DIR)/include

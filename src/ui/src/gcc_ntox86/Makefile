#
# Application Makefile   
#
ifdef DBG
DBG_RELEASE = -O3 -fomit-frame-pointer
DBG_DEBUG = -O -gdwarf-2
DEBUG=$(DBG_$(DBG))
endif

ifndef DEBUG
DEBUG = -g
endif

ifndef OPTIM
OPTIM = -O
endif

include	../abOfiles
include ../indOfiles

include	../abSfiles
include ../indSfiles

include	../abHfiles
include ../indHfiles

include	../abWfiles
include	../abLfiles

PLATFORM=gcc_ntox86
PHABOPTS= -l socket

CC  = QCC -O2
CXX = QCC -O2 -I $(HEADERSDIR) -I.
LD  = QCC

MROK = ../../../..
ECP_DIR = $(MROK)/src/ecp
ECP_MP_DIR = $(MROK)/src/ecp_mp
HEADERSDIR = $(MROK)/include
LIBDIR = $(MROK)/lib
BINDIR = $(MROK)/bin

CFLAGS  = -V$(PLATFORM) -w8 $(OPTIM) $(OPTIM) $(DEBUG) $(PHABOPTS) -I $(HEADERSDIR) -I.
LDFLAGS = -V$(PLATFORM) -Bstatic $(DEBUG) -l phexlib $(ABLIB) $(PHABOPTS)
SDFLAGS = -V$(PLATFORM) $(DEBUG) -l phexlib $(ABLIB) $(PHABOPTS)

MRROCOBJ = \
	$(ECP_DIR)/irp6_on_track/ecp_local.o \
	$(ECP_DIR)/irp6_postument/ecp_local.o \
	$(ECP_DIR)/irp6_mechatronika/ecp_local.o\
	$(ECP_DIR)/conveyor/ecp_local.o \
	$(ECP_DIR)/speaker/ecp_local.o \
	$(ECP_DIR)/common/libecp.a $(LIBDIR)/libmrrocpp.a

MYOBJ = ui_ecp_r_irp6_mechatronika.o \
	ui_ecp_r_conveyor.o ui_ecp_r_speaker.o \
	ui_ecp_r_irp6_common.o ui.o

VPATH = ..

#
# Application Program
#
#ui_ecp_r_irp6_on_track.o ui_ecp_r_irp6_postument.o

shr: ui

ui:	$(ABOBJ) $(MYOBJ) $(MRROCOBJ)
		$(LD) $(ABOBJ) $(MYOBJ) $(MRROCOBJ) $(SDFLAGS) -M -o ui
		usemsg ui ../Usemsg
		phabbind ui $(ABMOD)

app:	$(ABOBJ) $(MYOBJ) $(MRROCOBJ)
		$(LD) $(ABOBJ) $(MYOBJ) $(MRROCOBJ) $(LDFLAGS) -M -o ui
		usemsg ui ../Usemsg
		phabbind ui $(ABMOD)

clean:
		@rm -f *.o *.err *~ *.map ui

install: ui
		-cp ui $(MROK)/bin/

#
# Headers
#

$(ABOBJ) $(MYOBJ):	$(MYHDR)

#
# Prototype Generation
#

proto:
		@rm -f proto.h
		approto -p $(ABSRC) $(MYSRC) > proto.h

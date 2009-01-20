#
# Application Makefile   
#

include	../abOfiles
include ../indOfiles

include	../abSfiles
include ../indSfiles

include	../abHfiles
include ../indHfiles

include	../abWfiles
include	../abLfiles

PHABOPTS= -l socket

HOMEDIR = ../../../..
include $(HOMEDIR)/common.mk
HEADERSDIR = $(HOMEDIR)/include

CXXFLAGS += -I $(HEADERSDIR) -I.
LDFLAGS += -Bstatic -l phexlib $(ABLIB) $(PHABOPTS)
SDFLAGS = -l phexlib $(ABLIB) $(PHABOPTS)

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
		-${INSTALL} ui $(HOMEDIR)/bin/

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
		
-include ../.depend

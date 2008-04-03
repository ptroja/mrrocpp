MYSRC = ../fun.cc ../fun_r_conveyor.cc ../fun_r_irp6_mechatronika.cc ../fun_r_irp6_on_track.cc ../fun_r_irp6_postument.cc ../fun_r_speaker.cc ../timer.cc ../ui_init.cc

#
# Application Makefile   
#

PHABOPTS= -l socket -lm

HOMEDIR = ../../../..
include $(HOMEDIR)/common.mk
ECP_DIR = $(HOMEDIR)/src/ecp
UIDIR=$(HOMEDIR)/src/ui
UISRCDIR=$(HOMEDIR)/src/ui/src

HEADERSDIR = $(HOMEDIR)/include

CXXFLAGS += -I $(HEADERSDIR) -I.
LDFLAGS += -Bstatic $(PHABOPTS)
SDFLAGS = $(PHABOPTS)

MRROCOBJ = \
	$(ECP_DIR)/irp6_on_track/ecp_local.o \
	$(ECP_DIR)/irp6_postument/ecp_local.o \
	$(ECP_DIR)/irp6_mechatronika/ecp_local.o\
	$(ECP_DIR)/conveyor/ecp_local.o \
	$(ECP_DIR)/speaker/ecp_local.o \
	$(ECP_DIR)/common/libecp.a $(LIBDIR)/libmrrocpp.a

MYOBJ = $(UISRCDIR)/gcc_ntox86/ui_ecp_r_irp6_mechatronika.o \
	$(UISRCDIR)/gcc_ntox86/ui_ecp_r_conveyor.o \
	$(UISRCDIR)/gcc_ntox86/ui_ecp_r_speaker.o \
	$(UISRCDIR)/gcc_ntox86/ui_ecp_r_irp6_common.o \
	$(UISRCDIR)/gcc_ntox86/ui.o \
	../fun.o ../fun_r_conveyor.o ../fun_r_irp6_mechatronika.o ../fun_r_irp6_on_track.o ../fun_r_irp6_postument.o ../fun_r_speaker.o ../timer.o ../ui_init.o

VPATH = ..

#
# Application Program
#
#ui_ecp_r_irp6_on_track.o ui_ecp_r_irp6_postument.o

shr: ui_server

ui_server:	$(MYOBJ) $(MRROCOBJ)
		$(LD) $(MYOBJ) $(MRROCOBJ) $(SDFLAGS) -M -o ui_server -lang-c++

app:	$(ABOBJ) $(MYOBJ) $(MRROCOBJ)
		$(LD) $(MYOBJ) $(MRROCOBJ) $(LDFLAGS) -M -o ui_server -lang-c++

clean:
		@rm -f *.o *.err *~ *.map ui_server

install: ui_server
		-${INSTALL} ui_server $(HOMEDIR)/bin/

#
# Headers
#

$(MYOBJ):	$(MYHDR)

#
# Prototype Generation
#

proto:
		@rm -f proto.h
		approto -p $(MYSRC) > proto.h
		
-include ../.depend

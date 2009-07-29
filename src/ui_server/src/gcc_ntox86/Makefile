MYSRC = ../fun.cc ../fun_r_conveyor.cc ../fun_r_irp6_mechatronika.cc ../fun_r_irp6_on_track.cc ../fun_r_irp6_postument.cc ../fun_r_speaker.cc ../timer.cc ../ui_init.cc

#
# Application Makefile   
#

PHABOPTS= -l socket -lm

HOMEDIR = ../../../..
include $(HOMEDIR)/common.mk
UIDIR=$(HOMEDIR)/src/ui
UISRCDIR=$(HOMEDIR)/src/ui/src

HEADERSDIR = $(HOMEDIR)/include

CXXFLAGS += -I $(HEADERSDIR) -I.
LDFLAGS += -Bstatic $(PHABOPTS)
SDFLAGS = $(PHABOPTS) -L/opt/qnx641/target/qnx6/mrlib/lib -lxml2 -liconv

MRROCOBJ = \
	$(ECP_DIR)/irp6_on_track/ecp_r_irp6ot.o \
	$(ECP_DIR)/irp6_postument/ecp_r_irp6p.o \
	$(ECP_DIR)/irp6_mechatronika/ecp_r_irp6m.o\
	$(ECP_DIR)/conveyor/ecp_r_conv.o \
	$(ECP_DIR)/speaker/ecp_r_speaker.o \
	$(LIBDIR)/libecp.a $(LIBDIR)/libecp_mp.a  $(LIBDIR)/libmrrocpp.a

MYOBJ = $(UISRCDIR)/gcc_ntox86/ui_ecp_r_irp6_common.o \
	$(UISRCDIR)/gcc_ntox86/ui_ecp_r_conveyor.o \
	$(UISRCDIR)/gcc_ntox86/ui_ecp_r_speaker.o \
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
		@rm -f ../*.o *.err *~ *.map ui_server

install: ui_server
		-${INSTALL} ui_server $(BINDIR)

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

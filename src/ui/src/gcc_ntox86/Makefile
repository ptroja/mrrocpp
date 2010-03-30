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
KIN_OBJECTS = ${KIN_DIR}/irp6_on_track/libkinematicsirp6ot.a ${KIN_DIR}/irp6_postument/libkinematicsirp6p.a\
${KIN_DIR}/irp6ot_tfg/libkinematicsirp6ot_tfg.a ${KIN_DIR}/irp6p_tfg/libkinematicsirp6p_tfg.a\
 $(KIN_DIR)/common/libkinematics.a

include $(HOMEDIR)/common.mk
HEADERSDIR = $(HOMEDIR)/src

CPPFLAGS += -I $(HEADERSDIR) -I.
#LDFLAGS += -Bstatic -l phexlib $(ABLIB) $(PHABOPTS)
LDFLAGS += -l phexlib $(ABLIB) $(PHABOPTS)
#SDFLAGS = -l phexlib $(ABLIB) $(PHABOPTS) -L$(QNX_TARGET)/mrlib/lib -lxml2 -liconv

MRROCOBJ = \
	$(ECP_DIR)/irp6_on_track/ecp_r_irp6ot.o \
	$(ECP_DIR)/irp6_postument/ecp_r_irp6p.o \
	$(ECP_DIR)/irp6ot_tfg/ecp_r_irp6ot_tfg.o \
	$(ECP_DIR)/irp6p_tfg/ecp_r_irp6p_tfg.o \
	$(ECP_DIR)/irp6_mechatronika/ecp_r_irp6m.o\
	$(ECP_DIR)/polycrank/ecp_r_polycrank.o\
	$(ECP_DIR)/conveyor/ecp_r_conv.o \
	$(ECP_DIR)/speaker/ecp_r_speaker.o \
	$(ECP_DIR)/smb/ecp_r_smb.o \
	$(ECP_DIR)/spkm/ecp_r_spkm.o \
	$(ECP_DIR)/shead/ecp_r_shead.o \
	${KIN_OBJECTS}\
	$(LIBDIR)/libecp.a $(LIBDIR)/libecp_mp.a  $(LIBDIR)/libmrrocpp.a

MYOBJ = ui_ecp_r_speaker.o ui_ecp_r_common.o\
 ui_ecp_r_irp6_common.o ui_ecp_r_tfg_and_conv.o ui.o threads.o

VPATH = ..

#
# Application Program
#
#ui_ecp_r_irp6_on_track.o ui_ecp_r_irp6_postument.o

ifeq ($(BUILD_TARGET), linux)
shr: \
	ui.o \
	threads.o \
	ui_ecp_r_common.o \
	ui_ecp_r_irp6_common.o \
	ui_ecp_r_speaker.o \
	ui_ecp_r_tfg_and_conv.o
else
shr: ui

ui:	$(ABOBJ) $(MYOBJ) $(MRROCOBJ)
		$(LD) $(ABOBJ) $(MYOBJ) $(MRROCOBJ) $(LDFLAGS) -M -o ui
		usemsg ui ../Usemsg
		phabbind ui $(ABMOD)

app:	$(ABOBJ) $(MYOBJ) $(MRROCOBJ)
		$(LD) $(ABOBJ) $(MYOBJ) $(MRROCOBJ) $(LDFLAGS) -M -o ui
		usemsg ui ../Usemsg
		phabbind ui $(ABMOD)

install: ui
		-${INSTALL} ui $(BINDIR)
endif

clean:
		@rm -f *.o *.err *~ *.map ui

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

#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

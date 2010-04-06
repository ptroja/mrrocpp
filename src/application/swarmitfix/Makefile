#----------- swarmitfix catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}


MP_ROBOTS_OBJECTS = \
	../../mp/robot/mp_r_conveyor.o \
	../../mp/robot/mp_r_irp6_on_track.o \
	../../mp/robot/mp_r_irp6_postument.o \
	../../mp/robot/mp_r_speaker.o \
	../../mp/robot/mp_r_irp6_mechatronika.o \
	../../mp/robot/mp_r_polycrank.o \
	../../mp/robot/mp_r_spkm.o \
	../../mp/robot/mp_r_smb.o \
	../../mp/robot/mp_r_shead.o \
	../../mp/robot/mp_r_irp6ot_tfg.o \
	../../mp/robot/mp_r_irp6p_tfg.o \
	../../mp/robot/mp_r_manip_and_conv.o


BINARIES = ecp_smb

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

KIN_OBJECTS = ${KIN_DIR}/smb/libkinematicssmb.a $(KIN_DIR)/common/libkinematics.a

SHOBJ =  ${KIN_OBJECTS} $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}

ecp_smb: ecp_t_smb.o ../../ecp/smb/ecp_r_smb.o ../../ecp/common/libecp.a $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
	

#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ./sensor/*.o ./sensor/*.err ./generator/*.o ./generator/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

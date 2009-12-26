#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
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
	../../mp/robot/mp_r_manip_and_conv.o


BINARIES =		mp_multiplayer ecp_irp6ot_multiplayer 

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}

mp_multiplayer: mp_t_multiplayer.o ../../mp/libmprobots.a ../../mp/libmpmain.a ../../ecp_mp/libecp_mp.a $(MP_ROBOTS_OBJECTS) $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^


ecp_irp6ot_multiplayer: ecp_t_multiplayer_irp6ot.o ../../ecp/common/task/ecp_st_go.o ../../ecp/common/generator/ecp_g_smooth.o \
	../../ecp/common/generator/ecp_g_force.o  ../../ecp/common/generator/ecp_g_transparent.o \
	../../ecp/irp6_on_track/ecp_r_irp6ot.o ../../ecp/irp6_postument/ecp_r_irp6p.o ../../ecp/common/libecp.a $(SHOBJ)
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

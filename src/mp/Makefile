#----------- makefile of the MRROC++ frame system --------------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
MP_ROBOTS_OBJECTS = \
	robot/mp_r_conveyor.o \
	robot/mp_r_irp6_on_track.o \
	robot/mp_r_irp6_postument.o \
	robot/mp_r_speaker.o \
	robot/mp_r_irp6_mechatronika.o \
	robot/mp_r_polycrank.o \
	robot/mp_r_manip_and_conv.o
	
MP_MAIN_OBJECTS = task/mp_task.o robot/mp_robot.o mp_m.o \
	generator/mp_generator.o generator/mp_g_delay_ms_condition.o generator/mp_g_common.o
	
LIBMPROBOTS = libmprobots.a
LIBMPMAIN = libmpmain.a
	
BINARIES = \
	mp_c 

#	mp_pnml \
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBMPMAIN) $(LIBMPROBOTS) $(LIBDIR)/libecp_mp.a $(LIBDIR)/libmrrocpp.a

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${OBJECTS} ${BINARIES}

$(LIBMPROBOTS): $(MP_ROBOTS_OBJECTS)
	$(AR) $(ARFLAGS) $@ $?
	
$(LIBMPMAIN): $(MP_MAIN_OBJECTS)
	$(AR) $(ARFLAGS) $@ $?

#-----------------------------------------------------------------------------------------------------------------------#
mp_c: task/mp_t_c.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^  

mp_pnml: task/mp_t_pnml.o generator/mp_g_pnml.o \
	PNExec/libpnexec.a $(SHOBJ) -lxerces-c
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
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ${LIBMPROBOTS} ${LIBMPMAIN} ./task/*.o ./task/*.err ./generator/*.o ./generator/*.err ./robot/*.o ./robot/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

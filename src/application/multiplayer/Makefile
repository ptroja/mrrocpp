#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}


BINARIES =		vsp_time_sensor ecp_time 

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif

LIBMPROBOTS = libmprobots.a
LIBMPMAIN = libmpmain.a

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ../../ecp/irp6_on_track/ecp_r_irp6ot.o ../../ecp/irp6_postument/ecp_r_irp6p.o ../../ecp/common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}

vsp_time_sensor: $(VSPOBJS) vsp_time_sensor.o
	$(LD) $(LDFLAGS) -o $@  $^
	
ecp_time: ecp_mp_s_time.o ecp_t_time.o ecp_g_time.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

mp_multiplayer: task/mp_t_multiplayer.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
#ecp_irp6ot_multiplayer: task/ecp_t_multiplayer_irp6ot.o ../common/task/ecp_st_go.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_force.o \
#						../common/generator/ecp_g_transparent.o \
#						generator/ecp_g_rover.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o \
#						../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#


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

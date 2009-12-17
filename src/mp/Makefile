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
	
MP_MAIN_OBJECTS = mp_taught_in_pose.o task/mp_task.o robot/mp_robot.o mp_m.o \
	generator/mp_generator.o generator/mp_g_delay_ms_condition.o generator/mp_g_common.o
	
LIBMPROBOTS = libmprobots.a
LIBMPMAIN = libmpmain.a
	
BINARIES = \
	mp_c \
	mp_pr \
	mp_vf \
	mp_rc \
	mp_rcsc \
	mp_vis \
	mp_vis_nn \
	mp_vis_pbeclsac \
	mp_vis_sac \
	mp_vis_sac_lx \
	mp_rcs_test \
	mp_rcs \
	mp_pouring \
	mp_fsautomat \
	mp_multiplayer

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

mp_vf: task/mp_t_vf.o generator/mp_g_test.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_pr: task/mp_t_pr.o generator/mp_g_teach_in.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rc: task/mp_t_rc.o generator/mp_g_teach_in.o $(ECP_MPDIR)/transmitter/ecp_mp_tr_rc_windows.o generator/mp_g_vis.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcsc: task/mp_t_rcsc.o $(ECP_MPDIR)/transmitter/ecp_mp_tr_rc_windows.o generator/mp_g_teach_in.o generator/mp_g_vis.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis: task/mp_t_vis.o generator/mp_g_vis.o generator/mp_g_teach_in.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_kd: task/mp_t_kd.o generator/mp_g_teach_in.o generator/mp_g_force_kd.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_nn: task/mp_t_vis_nn.o generator/mp_g_teach_in.o generator/mp_g_vis_nn.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis_nn.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_pbeclsac: task/mp_t_vis_pbeclsac.o generator/mp_g_teach_in.o generator/mp_g_vis_pbeclsac.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_sac: task/mp_t_vis_sac.o generator/mp_g_teach_in.o generator/mp_g_vis_sac.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_sac_lx: task/mp_t_vis_sac_lx.o generator/mp_g_teach_in.o generator/mp_g_vis_sac_lx.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcs_test: task/mp_t_rcs_test.o $(ECP_MPDIR)/sensor/ecp_mp_s_rcs_korf.o $(ECP_MPDIR)/sensor/ecp_mp_s_rcs_kociemba.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_pouring: task/mp_t_pouring.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_fsautomat: task/mp_t_fsautomat.o \
	StateHeap.o State.o Condition.o Transition.o \
	$(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o \
	$(ECP_MPDIR)/sensor/ecp_mp_s_rcs_korf.o $(ECP_MPDIR)/sensor/ecp_mp_s_rcs_kociemba.o \
	$(ECP_MPDIR)/transmitter/ecp_mp_tr_rc_windows.o \
	generator/mp_g_teach_in.o generator/mp_g_force.o \
	generator/mp_g_vis.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis.o \
	CubeState.o SingleManipulation.o \
	$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_pnml: task/mp_t_pnml.o generator/mp_g_pnml.o \
	PNExec/libpnexec.a $(SHOBJ) -lxerces-c
	$(LD) $(LDFLAGS) -o $@ $^

mp_multiplayer: task/mp_t_multiplayer.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcs: task/mp_t_rcs.o generator/mp_g_teach_in.o $(ECP_MPDIR)/sensor/ecp_mp_s_rcs_korf.o $(ECP_MPDIR)/sensor/ecp_mp_s_rcs_kociemba.o $(ECP_MPDIR)/transmitter/ecp_mp_tr_rc_windows.o generator/mp_g_vis.o generator/mp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
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
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ${LIBMPROBOTS} ${LIBMPMAIN} \
	./task/*.o ./task/*.err ./generator/*.o ./generator/*.err ./robot/*.o ./robot/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

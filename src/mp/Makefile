#----------- makefile of the MRROC++ frame system --------------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
MP_ROBOTS_OBJECTS = \
	mp_r_conveyor.o \
	mp_r_irp6_on_track.o \
	mp_r_irp6_postument.o \
	mp_r_speaker.o \
	mp_r_irp6_mechatronika.o \
	mp_r_polycrank.o \
	mp_r_manip_and_conv.o
	
MP_MAIN_OBJECTS = mp.o mp_task.o mp_robot.o mp_m.o \
	mp_generator.o mp_delay_ms_condition.o
	
LIBMPROBOTS = libmprobots.a
LIBMPMAIN = libmpmain.a
	
BINARIES = \
	mp_c \
	mp_pr \
	mp_vf \
	mp_rc \
	mp_rcsc \
	mp_vis \
	mp_trm \
	mp_vis_nn \
	mp_vis_pbeclsac \
	mp_vis_sac \
	mp_vis_sac_lx \
	mp_rcs_test \
	mp_rcs \
	mp_haptic \
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
mp_c: mp_t_c.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^  

mp_vf: mp_t_vf.o mp_g_test.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(ECP_MPDIR)/ecp_mp_s_vis.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_pr: mp_t_pr.o mp_g_teach_in.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rc: mp_t_rc.o mp_g_teach_in.o $(ECP_MPDIR)/ecp_mp_tr_rc_windows.o mp_g_vis.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis.o $(ECP_MPDIR)/ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcsc: mp_t_rcsc.o $(ECP_MPDIR)/ecp_mp_tr_rc_windows.o mp_g_teach_in.o mp_g_vis.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis.o $(ECP_MPDIR)/ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_haptic: mp_t_haptic.o mp_g_teach_in.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis: mp_t_vis.o mp_g_vis.o mp_g_teach_in.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_kd: mp_t_kd.o mp_g_teach_in.o mp_g_force_kd.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_trm: mp_t_two_robots_measures.o mp_g_two_robots_measures.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_nn: mp_t_vis_nn.o mp_g_teach_in.o mp_g_vis_nn.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis_nn.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_pbeclsac: mp_t_vis_pbeclsac.o mp_g_teach_in.o mp_g_vis_pbeclsac.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_sac: mp_t_vis_sac.o mp_g_teach_in.o mp_g_vis_sac.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis_sac.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_sac_lx: mp_t_vis_sac_lx.o mp_g_teach_in.o mp_g_vis_sac_lx.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis_sac_lx.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcs_test: mp_t_rcs_test.o $(ECP_MPDIR)/ecp_mp_s_rcs_korf.o $(ECP_MPDIR)/ecp_mp_s_rcs_kociemba.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_pouring: mp_t_pouring.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_fsautomat: mp_t_fsautomat.o \
	StateHeap.o State.o Condition.o Transition.o \
	$(ECP_MPDIR)/ecp_mp_s_schunk.o \
	$(ECP_MPDIR)/ecp_mp_s_rcs_korf.o $(ECP_MPDIR)/ecp_mp_s_rcs_kociemba.o \
	$(ECP_MPDIR)/ecp_mp_tr_rc_windows.o \
	mp_g_teach_in.o mp_g_force.o \
	mp_g_vis.o $(ECP_MPDIR)/ecp_mp_s_vis.o \
	CubeState.o SingleManipulation.o \
	$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_pnml: mp_t_pnml.o mp_g_pnml.o \
	PNExec/libpnexec.a $(SHOBJ) -lxerces-c
	$(LD) $(LDFLAGS) -o $@ $^

mp_multiplayer: mp_t_multiplayer.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcs: mp_t_rcs.o mp_g_teach_in.o $(ECP_MPDIR)/ecp_mp_s_rcs_korf.o $(ECP_MPDIR)/ecp_mp_s_rcs_kociemba.o $(ECP_MPDIR)/ecp_mp_tr_rc_windows.o mp_g_vis.o mp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis.o $(ECP_MPDIR)/ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
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
	@rm -f *.o *.err ${BINARIES} ${LIBMPROBOTS} ${LIBMPMAIN}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

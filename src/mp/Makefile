#-----------ecp_mp/common catalog makefile of the MRROC++ frame system --------------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
MP_ROBOTS_OBJECTS = \
	mp_r_conveyor.o \
	mp_r_irp6_on_track.o \
	mp_r_irp6_postument.o \
	mp_r_speaker.o \
	mp_r_irp6_mechatronika.o \
	mp_r_irp6s_and_conv.o
	
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
	mp_sb \
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

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBMPMAIN) $(LIBMPROBOTS) $(LIBDIR)/libmrrocpp.a ../ecp/common/libecp.a

# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = $(ECP_MPDIR)
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${OBJECTS} ${BINARIES}

$(LIBMPROBOTS): $(MP_ROBOTS_OBJECTS)
	$(AR) $(ARFLAGS) $@ $?
	
$(LIBMPMAIN): $(MP_MAIN_OBJECTS)
	$(AR) $(ARFLAGS) $@ $?

#-----------------------------------------------------------------------------------------------------------------------#
mp_c: mp_t_c.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^  

mp_vf: mp_t_vf.o mp_g_test.o ecp_mp_s_schunk.o ecp_mp_s_vis.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_pr: mp_t_pr.o mp_g_teach_in.o mp_g_force.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rc: mp_t_rc.o mp_g_teach_in.o ecp_mp_tr_rc_windows.o mp_g_vis.o mp_g_force.o ecp_mp_s_vis.o ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcsc: mp_t_rcsc.o ecp_mp_tr_rc_windows.o mp_g_teach_in.o mp_g_vis.o mp_g_force.o ecp_mp_s_vis.o ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_haptic: mp_t_haptic.o mp_g_teach_in.o mp_g_force.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_sb: mp_t_sb.o mp_g_teach_in.o mp_g_force.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis: mp_t_vis.o mp_g_vis.o mp_g_teach_in.o mp_g_force.o ecp_mp_s_vis.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_kd: mp_t_kd.o mp_g_teach_in.o mp_g_force_kd.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_trm: mp_t_two_robots_measures.o mp_g_two_robots_measures.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_nn: mp_t_vis_nn.o mp_g_teach_in.o mp_g_vis_nn.o mp_g_force.o ecp_mp_s_vis_nn.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_pbeclsac: mp_t_vis_pbeclsac.o mp_g_teach_in.o mp_g_vis_pbeclsac.o mp_g_force.o ecp_mp_s_vis.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_sac: mp_t_vis_sac.o mp_g_teach_in.o mp_g_vis_sac.o mp_g_force.o ecp_mp_s_vis_sac.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_vis_sac_lx: mp_t_vis_sac_lx.o mp_g_teach_in.o mp_g_vis_sac_lx.o mp_g_force.o ecp_mp_s_vis_sac_lx.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcs_test: mp_t_rcs_test.o ecp_mp_s_rcs_korf.o ecp_mp_s_rcs_kociemba.o CubeState.o SingleManipulation.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_pouring: mp_t_pouring.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_t_fsautomat.o: mp_t_fsautomat.cc
	$(CXX) `xml2-config --cflags` -I/opt/include/ -I$(HOMEDIR)/include -c -o $@ $^

mp_fsautomat: mp_t_fsautomat.o State.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) `xml2-config --cflags` -I/opt/include/ -o $@ $^ `xml2-config --libs`

mp_multiplayer: mp_t_multiplayer.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

mp_rcs: mp_t_rcs.o mp_g_teach_in.o ecp_mp_s_rcs_korf.o ecp_mp_s_rcs_kociemba.o ecp_mp_tr_rc_windows.o mp_g_vis.o mp_g_force.o ecp_mp_s_vis.o ecp_mp_s_schunk.o CubeState.o SingleManipulation.o $(SHOBJ)
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

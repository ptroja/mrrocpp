#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES =			ecp_pr \
				ecp_jarosz_irp6ot \
				ecp_vis \
				ecp_vislx \
				ecp_rover \
				ecp_jajko \
				ecp_kbochnia \
				ecp_plot \
				ecp_pt \
				ecp_playerjoy \
				ecp_progpanel \
				ecp_fr \
				ecp_kin_test_irp6ot \
				ecp_irp6ot_rcsc \
				ecp_irp6ot_pouring \
				ecp_irp6ot_multiplayer \
				ecp_irp6ot_legobrick \
				ecp_vis_pb_eol_sac_irp6ot \
				ecp_vis_pb_eih_irp6ot \
				ecp_vis_ib_eih_irp6ot \
				ecp_vis_weights_driven_irp6ot \
				ecp_draughts \
				ecp_spots_recognition \
				ecp_pw_kolo\
				ecp_haar \
				ecp_ttt \
				ecp_ellipse \
				ecp_robot_calibration \
			        ecp_wii \
			        ecp_wii_velocity \
			        ecp_wii_teach \
				ecp_t_box_irp6ot
ifneq ($(BUILD_TARGET), linux)
BINARIES+=			ecp_fct \
				ecp_tr \
				ecp_t_smooth2_test \
				ecp_t_grab_cube_irp6ot \
				ecp_t_kcz_test \
				ecp_t_calib_axzb
endif

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ecp_r_irp6ot.o ../common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
ecp_haar: ecp_t_haar_irp6ot.o ecp_g_rotate_gripper.o ../common/ecp_g_force.o ../common/ecp_generator_t.o ../common/ecp_g_smooth.o ../common/ecp_g_jarosz.o\
		../common/ecp_operator_reaction_condition.o ecp_vis_ib_eih_planar_irp6ot.o  $(ECP_MPDIR)/ecp_mp_s_cvfradia.o   $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_ttt: ecp_t_ttt.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o \
                ../common/ecp_operator_reaction_condition.o ../common/ecp_g_sleep.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_ellipse: ecp_t_ellipse.o ecp_g_ellipse.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o $(ECP_MPDIR)/ecp_mp_s_wiimote.o \
                ../common/ecp_operator_reaction_condition.o ../common/ecp_g_sleep.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_wii: ecp_t_wii.o ecp_g_wii.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o $(ECP_MPDIR)/ecp_mp_s_wiimote.o \
                ../common/ecp_operator_reaction_condition.o ../common/ecp_g_sleep.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_wii_velocity: ecp_t_wii_velocity.o ecp_g_wii_velocity.o ../common/ecp_g_force.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o $(ECP_MPDIR)/ecp_mp_s_wiimote.o \
                ../common/ecp_operator_reaction_condition.o ../common/ecp_g_sleep.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_wii_teach: ecp_t_wii_teach.o ../common/ecp_g_force.o ../irp6_on_track/ecp_g_wii_teach.o ../common/ecp_g_smooth2.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o $(ECP_MPDIR)/ecp_mp_s_wiimote.o \
                ../common/ecp_operator_reaction_condition.o ../common/ecp_g_sleep.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_pw_kolo: ecp_g_pw_kolo.o ecp_t_pw_kolo_irp6ot.o ../common/ecp_g_force.o ../common/ecp_st_go.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o\
		../common/ecp_operator_reaction_condition.o ../common/ecp_g_sleep.o ecp_g_pw_kolo.o  $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_draughts: ecp_t_draughts.o ../common/ecp_g_force.o ../common/ecp_st_go.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o\
		 ../common/ecp_operator_reaction_condition.o $(ECP_MPDIR)/ecp_mp_s_cvfradia.o ../common/ecp_g_sleep.o ecp_vis_ib_eih_planar_irp6ot.o\
		 $(ECP_MPDIR)/ecp_mp_tr_draughtsAI.o ecp_vis_ib_eih_follower_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_spots_recognition: ecp_t_spots_recognition.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o\
		../common/ecp_g_cvfradia.o $(ECP_MPDIR)/ecp_mp_s_cvfradia.o ../common/ecp_operator_reaction_condition.o $(ECP_MPDIR)/ecp_mp_s_schunk.o \
		../common/ecp_g_force.o ../common/ecp_matrix4x4.o ecp_camera_to_tool.o ecp_g_spots_recognition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_robot_calibration: ecp_t_robot_calibration.o ecp_g_robot_calibration.o $(ECP_MPDIR)/ecp_mp_s_pcbird.o $(ECP_MPDIR)/birdclient.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_kin_test_irp6ot: ecp_t_kin_test_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_pr: ../common/ecp_g_force.o ecp_t_pr_irp6ot.o $(ECP_MPDIR)/ecp_mp_s_schunk.o \
		../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_rcsc: ecp_t_rcsc_irp6ot.o ../common/ecp_g_force.o ../common/ecp_st_go.o ../common/ecp_g_smooth.o ../common/ecp_g_smooth2.o ../common/ecp_generator_t.o \
		../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o ecp_vis_ib_eih_object_tracker_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_jarosz_irp6ot: ecp_t_jarosz_irp6ot.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@  $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_kbochnia: ecp_t_kbochnia_irp6ot.o ../common/ecp_g_space.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@  $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_jajko: ecp_t_jajko_irp6ot.o ../common/ecp_g_force.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis: ecp_t_vis_irp6ot.o ecp_g_vis.o $(ECP_MPDIR)/ecp_mp_s_vis.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vislx: ecp_t_vislx_irp6ot.o ecp_g_vis_sac_lx.o ../common/ecp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis_sac_lx.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_rover: ecp_t_rover_irp6ot.o ecp_g_rover.o ../common/ecp_g_force.o $(ECP_MPDIR)/ecp_mp_s_vis_sac_lx.o $(ECP_MPDIR)/ecp_mp_s_schunk.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_nurbs: ecp_t_ts_irp6ot.o ecp_g_nurbs.o $(SHOBJ) 
#	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_fct: ecp_t_fct_irp6ot.o ecp_g_fctg.o $(ECP_MPDIR)/ecp_mp_s_force.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_tr: ecp_t_tr_irp6ot.o ecp_g_trg.o ecp_g_rsc.o $(ECP_MPDIR)/ecp_mp_s_force.o $(ECP_MPDIR)/ecp_mp_s_digital_scales.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_pt: ecp_t_pt_irp6ot.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_playerjoy: ecp_t_playerjoy_irp6ot.o ../common/ecp_g_pjg.o $(ECP_MPDIR)/ecp_mp_tr_player.o $(LIBDIR)/libplayerc.a $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_progpanel: ecp_t_progpanel_irp6ot.o ../common/ecp_g_pp.o $(ECP_MPDIR)/ecp_mp_s_pp.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_fr: ecp_t_fr_irp6ot.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_pouring: ecp_t_pouring_irp6ot.o ../common/ecp_st_go.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o \
	../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_fsautomat: ../common/ecp_t_fsautomat.o ../../ecp_mp/Trajectory.o ../common/ecp_g_force.o ../common/ecp_st_go.o ../common/ecp_g_smooth.o \
	../common/ecp_generator_t.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_plot: ecp_t_plot_irp6ot.o ../common/ecp_g_plot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_tzu_cs: ecp_t_tzu_fs.o ecp_g_tzu_cs.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_legobrick: ecp_t_legobrick_irp6ot.o ../common/ecp_g_smooth.o ../common/ecp_g_jarosz.o ../common/ecp_g_force.o \
						../common/ecp_operator_reaction_condition.o ../common/ecp_generator_t.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_multiplayer: ecp_t_multiplayer_irp6ot.o ../common/ecp_st_go.o $(ECP_MPDIR)/ecp_mp_s_schunk.o ../common/ecp_g_smooth.o ../common/ecp_g_force.o \
						../common/ecp_generator_t.o \
						ecp_g_rover.o $(ECP_MPDIR)/ecp_mp_s_vis_sac_lx.o \
						../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_pb_eol_sac_irp6ot: ecp_t_vis_pb_eol_sac_irp6ot.o ecp_vis_pb_eol_sac_irp6ot.o \
						$(ECP_MPDIR)/ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_pb_eih_irp6ot: ecp_t_vis_pb_eih_irp6ot.o ecp_vis_pb_eih_irp6ot.o \
						$(ECP_MPDIR)/ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_weights_driven_irp6ot: ecp_t_vis_weights_driven_irp6ot.o ecp_vis_weights_driven_irp6ot.o ecp_vis_pb_eol_sac_irp6ot.o \
						ecp_vis_pb_eih_irp6ot.o ecp_vis_ib_eih_irp6ot.o $(ECP_MPDIR)/ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_ib_eih_irp6ot: ecp_t_vis_ib_eih_irp6ot.o ecp_vis_ib_eih_irp6ot.o \
						$(ECP_MPDIR)/ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_box_irp6ot: ecp_t_box_irp6ot.o ../common/ecp_g_smooth.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_smooth2_test: ecp_t_smooth2_test.o ../common/ecp_g_smooth2.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_grab_cube_irp6ot: ecp_vis_ib_eih_object_tracker_irp6ot.o ecp_t_grab_cube_irp6ot.o ../common/ecp_g_smooth2.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_kcz_test: ecp_t_kcz_test.o ../common/ecp_g_smooth2.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_calib_axzb: ../common/ecp_t_calib_axzb.o ../common/ecp_t_calibration.o ../common/ecp_g_smooth2.o ../common/ecp_generator_t.o ../common/ecp_g_jarosz.o ../common/ecp_operator_reaction_condition.o $(SHOBJ)
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
	@rm -f *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

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
				ecp_t_mboryn \
				#ecp_irp6ot_pouring \
				#ecp_irp6ot_multiplayer \
				#ecp_irp6ot_legobrick \
				ecp_vis_pb_eol_sac_irp6ot \
				ecp_vis_pb_eih_irp6ot \
				ecp_vis_ib_eih_irp6ot \
				ecp_vis_weights_driven_irp6ot \
				ecp_draughts \
				#ecp_pw_kolo\
				#ecp_haar \
				#ecp_ttt \
				#ecp_t_box_irp6ot
				
ifneq ($(BUILD_TARGET), linux)
BINARIES+=		ecp_t_smooth2_test \
				ecp_t_grab_cube_irp6ot \
				ecp_t_calib_axzb \
				ecp_t_calib_axxb
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
#ecp_haar: task/ecp_t_haar_irp6ot.o generator/ecp_g_rotate_gripper.o ../common/generator/ecp_g_force.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_jarosz.o\
#		../common/generator/ecp_g_operator_reaction_condition.o generator/ecp_g_vis_ib_eih_planar_irp6ot.o  $(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o   $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
#ecp_ttt: task/ecp_t_ttt.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o \
#                ../common/generator/ecp_g_operator_reaction_condition.o ../common/generator/ecp_g_sleep.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
#ecp_pw_kolo: generator/ecp_g_pw_kolo.o task/ecp_t_pw_kolo_irp6ot.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o\
#		../common/generator/ecp_g_operator_reaction_condition.o ../common/generator/ecp_g_sleep.o generator/ecp_g_pw_kolo.o  $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_draughts: task/ecp_t_draughts.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o\
		 ../common/generator/ecp_g_operator_reaction_condition.o $(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o ../common/generator/ecp_g_sleep.o generator/ecp_g_vis_ib_eih_planar_irp6ot.o\
		 $(ECP_MPDIR)/transmitter/ecp_mp_tr_draughtsAI.o generator/ecp_g_vis_ib_eih_follower_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_kin_test_irp6ot: task/ecp_t_kin_test_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_pr: ../common/generator/ecp_g_force.o task/ecp_t_pr_irp6ot.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o \
		../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_rcsc: task/ecp_t_rcsc_irp6ot.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_t.o \
		../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o generator/ecp_g_vis_ib_eih_object_tracker_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_jarosz_irp6ot: task/ecp_t_jarosz_irp6ot.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@  $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_kbochnia: task/ecp_t_kbochnia_irp6ot.o ../common/generator/ecp_g_space.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@  $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_jajko: task/ecp_t_jajko_irp6ot.o ../common/generator/ecp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis: task/ecp_t_vis_irp6ot.o generator/ecp_g_vis.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vislx: task/ecp_t_vislx_irp6ot.o generator/ecp_g_vis_sac_lx.o ../common/generator/ecp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_rover: task/ecp_t_rover_irp6ot.o generator/ecp_g_rover.o ../common/generator/ecp_g_force.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_nurbs: task/ecp_t_ts_irp6ot.o generator/ecp_g_nurbs.o $(SHOBJ) 
#	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_pt: task/ecp_t_pt_irp6ot.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_playerjoy: task/ecp_t_playerjoy_irp6ot.o ../common/generator/ecp_g_pjg.o $(ECP_MPDIR)/transmitter/ecp_mp_tr_player.o $(LIBDIR)/libplayerc.a $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_progpanel: task/ecp_t_progpanel_irp6ot.o ../common/generator/ecp_g_pp.o $(ECP_MPDIR)/sensor/ecp_mp_s_pp.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_fr: task/ecp_t_fr_irp6ot.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_irp6ot_pouring: task/ecp_t_pouring_irp6ot.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o \
#	../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_irp6ot_fsautomat: ../common/task/ecp_t_fsautomat.o ../../ecp_mp/Trajectory.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_smooth.o \
#	../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_plot: task/ecp_t_plot_irp6ot.o ../common/generator/ecp_g_plot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_tzu_cs: task/ecp_t_tzu_fs.o generator/ecp_g_tzu_cs.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_irp6ot_legobrick: task/ecp_t_legobrick_irp6ot.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_force.o \
#						../common/generator/ecp_g_operator_reaction_condition.o ../common/generator/ecp_g_t.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_irp6ot_multiplayer: task/ecp_t_multiplayer_irp6ot.o ../common/task/ecp_st_go.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_force.o \
#						../common/generator/ecp_g_t.o \
#						generator/ecp_g_rover.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o \
#						../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_pb_eol_sac_irp6ot: task/ecp_t_vis_pb_eol_sac_irp6ot.o generator/ecp_g_vis_pb_eol_sac_irp6ot.o \
						$(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_pb_eih_irp6ot: task/ecp_t_vis_pb_eih_irp6ot.o generator/ecp_g_vis_pb_eih_irp6ot.o \
						$(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_weights_driven_irp6ot: task/ecp_t_vis_weights_driven_irp6ot.o generator/ecp_g_vis_weights_driven_irp6ot.o generator/ecp_g_vis_pb_eol_sac_irp6ot.o \
						generator/ecp_g_vis_pb_eih_irp6ot.o generator/ecp_g_vis_ib_eih_irp6ot.o $(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_ib_eih_irp6ot: task/ecp_t_vis_ib_eih_irp6ot.o generator/ecp_g_vis_ib_eih_irp6ot.o \
						$(ECP_MPDIR)/sensor/ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_box_irp6ot: task/ecp_t_box_irp6ot.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_smooth2_test: task/ecp_t_smooth2_test.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_grab_cube_irp6ot: generator/ecp_g_vis_ib_eih_object_tracker_irp6ot.o generator/ecp_g_vis_ib_eih_wrist_turner_irp6ot.o ../common/generator/ecp_g_force.o task/ecp_t_grab_cube_irp6ot.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o $(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_calib_axzb: ../common/task/ecp_t_calib_axzb.o ../common/task/ecp_t_calibration.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ -lgsl -lgslcblas $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_calib_axxb: ../common/task/ecp_t_calib_axxb.o ../common/task/ecp_t_calibration.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ -lgsl -lgslcblas $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_mboryn: task/ecp_t_mboryn.o generator/ecp_g_mboryn.o  $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
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
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ./task/*.o ./task/*.err ./generator/*.o ./generator/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

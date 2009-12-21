#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES =		ecp_playerjoy \
				ecp_irp6ot_rcsc \
				ecp_draughts \
				#ecp_irp6ot_pouring \
				#ecp_irp6ot_multiplayer \
				#ecp_irp6ot_legobrick \
				ecp_vis_pb_eol_sac_irp6ot \
				ecp_vis_pb_eih_irp6ot \
				ecp_vis_ib_eih_irp6ot \
				ecp_vis_weights_driven_irp6ot \
				#ecp_pw_kolo\
				#ecp_haar \
				#ecp_ttt \
				#ecp_t_box_irp6ot
				
ifneq ($(BUILD_TARGET), linux)
BINARIES+=		ecp_t_smooth2_test \
				ecp_t_grab_cube_irp6ot

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
#ecp_haar: task/ecp_t_haar_irp6ot.o generator/ecp_g_rotate_gripper.o ../common/generator/ecp_g_force.o ../common/generator/ecp_g_transparent.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_jarosz.o\
#		../common/generator/ecp_g_operator_reaction_condition.o generator/ecp_g_vis_ib_eih_planar_irp6ot.o  $(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o   $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
#ecp_ttt: task/ecp_t_ttt.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_transparent.o ../common/generator/ecp_g_jarosz.o \
#                ../common/generator/ecp_g_operator_reaction_condition.o ../common/generator/ecp_g_sleep.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
#ecp_pw_kolo: generator/ecp_g_pw_kolo.o task/ecp_t_pw_kolo_irp6ot.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_transparent.o ../common/generator/ecp_g_jarosz.o\
#		../common/generator/ecp_g_operator_reaction_condition.o ../common/generator/ecp_g_sleep.o generator/ecp_g_pw_kolo.o  $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_draughts: task/ecp_t_draughts.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_transparent.o ../common/generator/ecp_g_jarosz.o\
		 ../common/generator/ecp_g_operator_reaction_condition.o $(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o ../common/generator/ecp_g_sleep.o generator/ecp_g_vis_ib_eih_planar_irp6ot.o\
		 $(ECP_MPDIR)/transmitter/ecp_mp_tr_draughtsAI.o generator/ecp_g_vis_ib_eih_follower_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^



#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_rcsc: task/ecp_t_rcsc_irp6ot.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_transparent.o \
		../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o generator/ecp_g_vis_ib_eih_object_tracker_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^



#-----------------------------------------------------------------------------------------------------------------------#
ecp_playerjoy: task/ecp_t_playerjoy_irp6ot.o ../common/generator/ecp_g_pjg.o $(ECP_MPDIR)/transmitter/ecp_mp_tr_player.o $(LIBDIR)/libplayerc.a $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^



#-----------------------------------------------------------------------------------------------------------------------#
#ecp_irp6ot_fsautomat: ../common/task/ecp_t_fsautomat.o ../../ecp_mp/Trajectory.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_smooth.o \
#	../common/generator/ecp_g_transparent.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_irp6ot_multiplayer: task/ecp_t_multiplayer_irp6ot.o ../common/task/ecp_st_go.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_force.o \
#						../common/generator/ecp_g_transparent.o \
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
ecp_t_smooth2_test: task/ecp_t_smooth2_test.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_transparent.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_grab_cube_irp6ot: generator/ecp_g_vis_ib_eih_object_tracker_irp6ot.o generator/ecp_g_vis_ib_eih_wrist_turner_irp6ot.o ../common/generator/ecp_g_force.o task/ecp_t_grab_cube_irp6ot.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_transparent.o ../common/generator/ecp_g_jarosz.o $(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
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

#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES =		ecp_pr \
				ecp_jarosz_irp6ot \
				ecp_vis \
				ecp_vislx \
				ecp_rover \
				ecp_jajko \
				ecp_kbochnia \
				ecp_fct \
				ecp_tr \
				ecp_plot \
				ecp_pt \
				ecp_playerjoy \
				ecp_progpanel \
				ecp_fr \
				ecp_kin_test_irp6ot \
				ecp_irp6ot_rcsc \
				ecp_irp6ot_pouring \
				ecp_irp6ot_fsautomat \
				ecp_irp6ot_multiplayer \
				ecp_irp6ot_legobrick \
				ecp_vis_pb_eol_sac_irp6ot \
				ecp_vis_pb_eih_irp6ot \
				ecp_vis_ib_eih_irp6ot \
				ecp_vis_weights_driven_irp6ot \
				ecp_tb \
				ecp_spots_recognition\
				ecp_pw\
				ecp_pw_scena \
				ecp_ttt \
				ecp_ellipse

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ecp_local.o ../common/libecp.a $(LIBDIR)/libmrrocpp.a
VPATH = ../common $(ECP_MPDIR)
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
ecp_pw_scena: ecp_g_pw_scena.o ecp_t_pw_scena_irp6ot.o  ecp_generator_t.o ecp_g_smooth.o ecp_g_jarosz.o\
		ecp_operator_reaction_condition.o  ecp_mp_s_cvfradia.o  ecp_g_pw_scena.o  $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_ttt: ecp_t_ttt.o ecp_g_smooth.o ecp_generator_t.o ecp_g_jarosz.o \
                ecp_operator_reaction_condition.o ecp_g_sleep.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_ellipse: ecp_t_ellipse.o ecp_g_ellipse.o ecp_g_smooth.o ecp_generator_t.o ecp_g_jarosz.o ecp_mp_s_wiimote.o \
                ecp_operator_reaction_condition.o ecp_g_sleep.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_pw: ecp_g_pw_kolo.o ecp_t_pw_irp6ot.o ecp_g_force.o ecp_st_go.o ecp_g_smooth.o ecp_generator_t.o ecp_g_jarosz.o\
		ecp_operator_reaction_condition.o ecp_g_sleep.o ecp_g_pw_kolo.o  $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_tb: ecp_t_tb_irp6ot.o ecp_g_force.o ecp_st_go.o ecp_g_smooth.o ecp_generator_t.o ecp_g_jarosz.o\
		 ecp_operator_reaction_condition.o ecp_mp_s_cvfradia.o ecp_g_sleep.o ecp_g_tbtest.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_spots_recognition: ecp_t_spots_recognition.o ecp_g_smooth.o ecp_generator_t.o ecp_g_jarosz.o\
		ecp_g_cvfradia.o ecp_mp_s_cvfradia.o ecp_operator_reaction_condition.o ecp_mp_s_schunk.o \
		ecp_matrix4x4.o ecp_g_spots_recognition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_kin_test_irp6ot: ecp_t_kin_test_irp6ot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_pr: ecp_g_force.o ecp_t_pr_irp6ot.o ecp_mp_s_schunk.o \
		ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_rcsc: ecp_t_rcsc_irp6ot.o ecp_g_force.o ecp_st_go.o ecp_g_smooth.o ecp_generator_t.o \
		ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_jarosz_irp6ot: ecp_t_jarosz_irp6ot.o ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@  $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_kbochnia: ecp_t_kbochnia_irp6ot.o ecp_g_space.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@  $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_jajko: ecp_t_jajko_irp6ot.o ecp_g_force.o ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis: ecp_t_vis_irp6ot.o ecp_g_vis.o ecp_mp_s_vis.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vislx: ecp_t_vislx_irp6ot.o ecp_g_vis_sac_lx.o ecp_g_force.o ecp_mp_s_vis_sac_lx.o ecp_mp_s_schunk.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_rover: ecp_t_rover_irp6ot.o ecp_g_rover.o ecp_g_force.o ecp_mp_s_vis_sac_lx.o ecp_mp_s_schunk.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_nurbs: ecp_t_ts_irp6ot.o ecp_g_nurbs.o $(SHOBJ) 
#	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_fct: ecp_t_fct_irp6ot.o ecp_g_fctg.o ecp_mp_s_force.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_tr: ecp_t_tr_irp6ot.o ecp_g_trg.o ecp_g_rsc.o ecp_mp_s_force.o ecp_mp_s_digital_scales.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_pt: ecp_t_pt_irp6ot.o ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_playerjoy: ecp_t_playerjoy_irp6ot.o ../common/ecp_g_pjg.o ecp_mp_tr_player.o $(LIBDIR)/libplayerc.a $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_progpanel: ecp_t_progpanel_irp6ot.o ../common/ecp_g_pp.o ecp_mp_s_pp.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_fr: ecp_t_fr_irp6ot.o ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_pouring: ecp_t_pouring_irp6ot.o ecp_st_go.o ecp_g_smooth.o ecp_generator_t.o ecp_g_jarosz.o \
	ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_fsautomat: ecp_t_fsautomat_irp6ot.o ../../mp/Trajectory.o ecp_g_force.o ecp_st_go.o ecp_g_smooth.o \
	ecp_generator_t.o ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_plot: ecp_t_plot_irp6ot.o ecp_g_plot.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_tzu_cs: ecp_t_tzu_fs.o ecp_g_tzu_cs.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_legobrick: ecp_t_legobrick_irp6ot.o ecp_g_smooth.o ecp_g_jarosz.o ecp_g_force.o \
						ecp_operator_reaction_condition.o ecp_generator_t.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6ot_multiplayer: ecp_t_multiplayer_irp6ot.o ecp_st_go.o ecp_mp_s_schunk.o ecp_g_smooth.o ecp_g_force.o \
						ecp_generator_t.o \
						ecp_g_rover.o ecp_mp_s_vis_sac_lx.o \
						ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_pb_eol_sac_irp6ot: ecp_t_vis_pb_eol_sac_irp6ot.o ecp_vis_pb_eol_sac_irp6ot.o \
						ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_pb_eih_irp6ot: ecp_t_vis_pb_eih_irp6ot.o ecp_vis_pb_eih_irp6ot.o \
						ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^
#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_weights_driven_irp6ot: ecp_t_vis_weights_driven_irp6ot.o ecp_vis_weights_driven_irp6ot.o ecp_vis_pb_eol_sac_irp6ot.o \
						ecp_vis_pb_eih_irp6ot.o ecp_vis_ib_eih_irp6ot.o ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
ecp_vis_ib_eih_irp6ot: ecp_t_vis_ib_eih_irp6ot.o ecp_vis_ib_eih_irp6ot.o \
						ecp_mp_s_vis_sac_lx.o $(SHOBJ) 
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

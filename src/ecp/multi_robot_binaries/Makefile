#-----------ecp_mp/common catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES = ecp_tran \
	ecp_t \
	ecp_nalewanie \
	ecp_sk \
	ecp_tw \
	ecp_time \
	ecp_pawel \
	ecp_cvfradia \
	ecp_tzu \
	ecp_tzu_test \
	ecp_pcbird \
	ecp_visioncoordinates \
	ecp_eihcalibration \
	ecp_mam \
	ecp_fsautomat \
	ecp_t_kcz_force
	

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ../common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}

#-----------------------------------------------------------------------------------------------------------------------#

ecp_tran: \
		../common/ecp_t_tran.o \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../conveyor/ecp_r_conv.o \
		../speaker/ecp_r_speaker.o \
		../irp6_mechatronika/ecp_r_irp6m.o \
		../polycrank/ecp_r_polycrank.o \
		../common/ecp_generator_t.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_mam: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_mam.o \
		../common/ecp_g_mam.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_digital_scales.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_sk: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_sk.o \
		../common/ecp_g_force.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_tw: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_tw.o \
		../common/ecp_g_force.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_t: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../irp6_mechatronika/ecp_r_irp6m.o  \
		../common/ecp_t_teach.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_nalewanie: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_nalewanie.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_time: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_time.o \
		../common/ecp_t_time.o \
		../common/ecp_g_time.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_ttt: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../common/ecp_t_ttt.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

ecp_ellipse: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../common/ecp_t_ellipse.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_ellipse.o \		
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

ecp_wii: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../common/ecp_t_wii.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_wii.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

ecp_fsautomat: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_fsautomat.o \
		../../ecp_mp/Trajectory.o \
		../common/ecp_g_force.o \
		../common/ecp_st_go.o \
		../common/ecp_g_smooth.o \
		../common/ecp_generator_t.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
	
ecp_pawel: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_pawel.o \
		../common/ecp_g_pawel.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_pawel.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_cvfradia: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_cvfradia.o \
		../common/ecp_g_cvfradia.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_visioncoordinates: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_visioncoordinates.o \
		../common/ecp_g_visioncoordinates.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_tzu: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_tzu_fs.o \
		../common/ecp_g_force.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

ecp_tzu_test: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_tzu_test.o \
		../common/ecp_g_force.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^  

ecp_pcbird: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_pcbird.o \
		../common/ecp_g_cvfradia.o \
		$(ECP_MPDIR)/birdclient.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_pcbird.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
	
ecp_eihcalibration: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_eihcalibration.o \
		../common/ecp_g_eihcalibration.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_smooth2.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_g_force.o \
		../common/ecp_operator_reaction_condition.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
	
ecp_t_kcz_force: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/ecp_t_kcz_force.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_smooth2.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_g_force.o \
		../common/ecp_operator_reaction_condition.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o \
		$(ECP_MPDIR)/birdclient.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_pcbird.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -lgsl -lgslcblas -o $@ $^ 

#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: $(BINARIES)
	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.a *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

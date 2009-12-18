#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES =		ecp_t_kcz_force \
				ecp_t_kcz_test \
				ecp_eihcalibration\
				ecp_t_calib_axzb \
				ecp_t_calib_axxb

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif


#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ../../ecp_mp/libecp_mp.a ../../ecp_mp/ecp_mp_robot.o ../../ecp/irp6_on_track/ecp_r_irp6ot.o ../../ecp/irp6_postument/ecp_r_irp6p.o ../../ecp/common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
ecp_t_calib_axzb: \
		ecp_t_calib_axzb.o \
		ecp_t_calibration.o \
		../../ecp/common/generator/ecp_g_smooth2.o \
		../../ecp/common/generator/ecp_g_t.o \
		../../ecp/common/generator/ecp_g_jarosz.o \
		../../ecp/common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ -lgsl -lgslcblas $^

ecp_t_calib_axxb: \
		ecp_t_calib_axxb.o \
		ecp_t_calibration.o \
		../../ecp/common/generator/ecp_g_smooth2.o \
		../../ecp/common/generator/ecp_g_t.o \
		../../ecp/common/generator/ecp_g_jarosz.o \
		../../ecp/common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ -lgsl -lgslcblas $^ 

ecp_eihcalibration: \
		../../ecp/irp6_on_track/ecp_r_irp6ot.o \
		../../ecp/irp6_postument/ecp_r_irp6p.o \
		ecp_t_eihcalibration.o \
		ecp_g_eihcalibration.o \
		../../ecp/common/generator/ecp_g_smooth2.o \
		../../ecp/common/generator/ecp_g_jarosz.o \
		../../ecp/common/generator/ecp_g_force.o \
		../../ecp/common/generator/ecp_g_operator_reaction_condition.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
	
ecp_t_kcz_force: \
		../../ecp/irp6_on_track/ecp_r_irp6ot.o \
		../../ecp/irp6_postument/ecp_r_irp6p.o \
		ecp_t_kcz_force.o \
		../../ecp/common/generator/ecp_g_smooth2.o \
		../../ecp/common/generator/ecp_g_jarosz.o \
		../../ecp/common/generator/ecp_g_force.o \
		../../ecp/common/generator/ecp_g_operator_reaction_condition.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o \
		$(ECP_MPDIR)/birdclient.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_pcbird.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -lgsl -lgslcblas -o $@ $^ 
	
ecp_t_kcz_test: \
		../../ecp/irp6_on_track/ecp_r_irp6ot.o \
		../../ecp/irp6_postument/ecp_r_irp6p.o \
		ecp_t_kcz_test.o \
		../../ecp/common/generator/ecp_g_smooth2.o \
		../../ecp/common/generator/ecp_g_jarosz.o \
		../../ecp/common/generator/ecp_g_force.o \
		../../ecp/common/generator/ecp_g_operator_reaction_condition.o \
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
install: ${BINARIES}
	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ./sensor/*.o ./sensor/*.err ./generator/*.o ./generator/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

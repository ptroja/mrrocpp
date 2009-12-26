#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES =		\
				ecp_t_axzb_eih \
				ecp_t_axzb_force \
				ecp_t_kcz_test

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
ecp_t_axzb_eih: \
        ecp_t_calib_axzb.o \
		ecp_t_calibration.o \
		ecp_st_acq_eih.o \
		ecp_st_acquisition.o \
        ecp_t_axzb_eih.o \
        ../../ecp/common/generator/ecp_g_smooth.o \
        ecp_g_eihcalibration.o \
        ecp_g_eih_nose_run.o \
        ../../ecp/common/generator/ecp_g_force.o \
        ../../ecp/common/generator/ecp_g_operator_reaction_condition.o \
        $(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o \
        $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ -lgsl -lgslcblas $^
	
ecp_t_axzb_force: \
		ecp_t_calib_axzb.o \
		ecp_t_calibration.o \
		ecp_st_acq_force.o \
		ecp_st_acquisition.o \
		ecp_t_axzb_force.o \
		../../ecp/common/generator/ecp_g_smooth.o \
		ecp_g_pcbird_nose_run.o \
		../../ecp/common/generator/ecp_g_force.o \
		$(ECP_MPDIR)/birdclient.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_pcbird.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ -lgsl -lgslcblas $^
	
ecp_t_kcz_test: \
		ecp_t_kcz_test.o \
		../../ecp/common/generator/ecp_g_smooth.o \
		$(ECP_MPDIR)/birdclient.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_pcbird.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ -lgsl -lgslcblas $^ 

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

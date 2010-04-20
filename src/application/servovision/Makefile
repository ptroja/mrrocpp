#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES = ecp_mboryn ecp_fradia_lag_test ecp_ib_eih_tester #ecp_pb_eih_tester

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk
KIN_OBJECTS = ${KIN_DIR}/irp6_on_track/libkinematicsirp6ot.a ${KIN_DIR}/irp6_postument/libkinematicsirp6p.a $(KIN_DIR)/common/libkinematics.a

SHOBJ = \
	../../ecp/irp6_on_track/ecp_r_irp6ot.o \
	../../ecp/irp6_postument/ecp_r_irp6p.o \
	${KIN_OBJECTS} \
	../../ecp/common/libecp.a \
	../../ecp_mp/libecp_mp.a \
	$(LIBDIR)/libmrrocpp.a

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
ecp_mboryn: 1st_ib_eih_2d/ecp_t_mboryn.o 1st_ib_eih_2d/ecp_g_mboryn.o $(SHOBJ) ../../ecp_mp/sensor/ecp_mp_s_cvfradia.o
	$(LD) $(LDFLAGS) -o $@ $^
	
ecp_fradia_lag_test: fradia_lag_test/ecp_t_fradia_lag_test.o $(SHOBJ) ../../ecp_mp/sensor/ecp_mp_s_cvfradia.o
	$(LD) $(LDFLAGS) -o $@ $^

ecp_ib_eih_tester: ecp_g_ib_eih.o ecp_g_visual_servo.o ib_eih_tester/ecp_t_ib_eih_tester.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
	
ecp_pb_eih_tester: ecp_g_pb_eih.o ecp_g_visual_servo.o pb_eih_tester/ecp_t_pb_eih_tester.o $(SHOBJ)
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
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ./sensor/*.o ./sensor/*.err ./generator/*.o ./generator/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

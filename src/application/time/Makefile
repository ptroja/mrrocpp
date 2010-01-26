#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}


ifeq ($(BUILD_TARGET), linux)
  VSPOBJS = ../../vsp/common/vsp_m_int_srr.o  $(LIBDIR)/libmrrocpp.a ../../vsp/common/vsp_sensor.o
else
  VSPOBJS = ../../vsp/common/vsp_m_int.o  $(LIBDIR)/libmrrocpp.a ../../vsp/common/vsp_sensor.o
endif

BINARIES =		vsp_time_sensor ecp_time 

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk
KIN_OBJECTS = ${KIN_DIR}/irp6_on_track/libkinematicsirp6ot.a ${KIN_DIR}/irp6_postument/libkinematicsirp6p.a $(KIN_DIR)/common/libkinematics.a

SHOBJ = ../../ecp/irp6_on_track/ecp_r_irp6ot.o ../../ecp/irp6_postument/ecp_r_irp6p.o ${KIN_OBJECTS} ../../ecp/common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}

vsp_time_sensor: $(VSPOBJS) vsp_time_sensor.o
	$(LD) $(LDFLAGS) -o $@  $^
	
ecp_time: ecp_mp_s_time.o ecp_t_time.o ecp_g_time.o $(SHOBJ)
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

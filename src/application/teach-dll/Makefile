#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES = hello.so generator.so ecp_teach-dll

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif


#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
KINEMATDIR = ${HOMEDIR}/src/kinematics
include $(HOMEDIR)/common.mk

KIN_OBJECTS = ${KINEMATDIR}/irp6_on_track/libkinematicsirp6ot.a $(KINEMATDIR)/common/libkinematics.a
SHOBJ =  ../../ecp_mp/libecp_mp.a ../../ecp_mp/ecp_mp_robot.o ../../ecp/irp6_on_track/ecp_r_irp6ot.o ../../ecp/irp6_postument/ecp_r_irp6p.o\
 ../../ecp/irp6_mechatronika/ecp_r_irp6m.o ${KIN_OBJECTS} ../../ecp/common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
ecp_teach-dll: ecp_t_teach.o ecp_g_teach.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
	
hello.so: hello.o
	$(CXX) $^ $(LDFLAGS) -shared -o $@
	
generator.so: ecp_g_teach.o $(SHOBJ)
	$(CXX) $^ $(LDFLAGS) -shared -o $@

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

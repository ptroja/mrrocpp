#-----------ecp_mp/common catalog makefile of the MRROC++ frame system -------------------------------------------------#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

ECP_MPOBJ_SHARED = ecp_mp_generator.o ecp_mp_sensor.o transmitter.o ecp_mp_task.o ecp_mp_robot.o Trajectory.o ecp_mp_smooth_taught_in_pose.o

ECP_MPOBJ = $(ECP_MPOBJ_SHARED) 


.PHONY: all clean install depend
#------------------------------------------------- Build rules ---------------------------------------------------------#

all: libecp_mp.a libecp_mp.so ${OBJECTS}

#-----------------------------------------------------------------------------------------------------------------------#

libecp_mp.a: $(ECP_MPOBJ)
	$(AR) $(ARFLAGS) $@ $? 
	${INSTALL} $@ $(LIBDIR)

libecp_mp.so: $(ECP_MPOBJ_SHARED)
	$(CXX) $^ $(LDFLAGS) -shared -o $@
	${INSTALL} $@ $(LIBDIR)

#-----------------------------------------------------------------------------------------------------------------------#


#---------------------------------Special object build rules (user made)------------------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules --------------------------------------------------------#
# install:
#	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.a *.o *.so *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

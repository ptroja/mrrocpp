#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

ECPOBJ_SHARED = \
	ECP_main_error.o \
	ecp_robot.o ecp_mp_robot.o \
	ecp_mp_generator.o \
	ecp_mp_task.o \
	../../ecp_mp/ecp_mp_smooth_taught_in_pose.o \
	../../ecp_mp/Trajectory.o \
	ecp_mp_sensor.o transmitter.o

ECPOBJ = $(ECPOBJ_SHARED) \
	ecp_m.o ecp_task.o \
	ecp_generator.o ecp_teach_in_generator.o \
	ecp_taught_in_pose.o \
	ecp_g_jarosz.o \
	ecp_g_smooth2.o ecp_smooth2_taught_in_pose.o \
	ecp_visual_servo_manager.o ecp_visual_servo.o 
		
# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = $(ECP_MPDIR)

#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: libecp.a libecp.so

#-----------------------------------------------------------------------------------------------------------------------#

libecp.a: $(ECPOBJ)
	$(AR) $(ARFLAGS) $@ $? 
	${INSTALL} $@ $(LIBDIR)

libecp.so: $(ECPOBJ_SHARED)
	$(CXX) $^ $(LDFLAGS) -shared -o $@
	${INSTALL} $@ $(LIBDIR)

#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
#install: $(BINARIES)
#	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.a *.o *.so *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

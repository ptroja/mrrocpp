#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

ECPOBJ_SHARED = ECP_main_error.o $(ECP_MPDIR)/ecp_mp_generator.o \
		ecp_robot.o \
		$(ECP_MPDIR)/ecp_mp_robot.o
ECPOBJ = $(ECPOBJ_SHARED) \
		ecp_m.o ecp_task.o $(ECP_MPDIR)/ecp_mp_task.o \
		ecp_generator.o ecp_teach_in_generator.o \
		ecp_visual_servo_manager.o ecp_visual_servo.o \
		ecp_taught_in_pose.o \
		$(ECP_MPDIR)/smooth_trajectory_pose.o $(ECP_MPDIR)/smooth2_trajectory_pose.o \
		ecp_g_smooth2.o \
		$(ECP_MPDIR)/ecp_mp_sensor.o $(ECP_MPDIR)/transmitter.o \
		$(ECP_MPDIR)/Trajectory.o
		
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: libecp.a libecp.so

#-----------------------------------------------------------------------------------------------------------------------#

libecp.a: $(ECPOBJ)
	$(AR) $(ARFLAGS) $@ $? 
	${INSTALL} $@ $(LIBDIR)

ifeq ($(BUILD_TARGET), linux)
LDFLAGS+=-export-dynamic
endif

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

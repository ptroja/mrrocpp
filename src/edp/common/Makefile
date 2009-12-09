#---------------- edp/common catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Should be modified only by developers not by users
HOMEDIR = ../../..
KINEMATDIR = ${HOMEDIR}/src/kinematics
include $(HOMEDIR)/common.mk

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

EDPOBJ = edp_m.o edp_effector.o\
	edp_e_manip.o edp_e_manip_and_conv.o \
	edp_force_sensor.o force.o \
	servo_gr.o servo_ms.o \
	master_trans_t_buffer.o vis_server.o reader.o \
	${KINEMATDIR}/common/transformer_error.o ${KINEMATDIR}/common/kinematic_model.o ${KINEMATDIR}/common/kinematics_manager.o \
	hi_rydz.o sg_irp6p_and_conv.o edp_irp6s_postument_track.o

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(OBJECTS) libedp.a

libedp.a: $(EDPOBJ)
	$(AR) $(ARFLAGS) $@ $?
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
#install:
#	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.a *.err 
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

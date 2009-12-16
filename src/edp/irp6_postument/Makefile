#------------- edp/irp6_postument catalog makefile of the MRROC++ frame system ---------------#
#======================================================#

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
KINEMATDIR = ${HOMEDIR}/src/kinematics
include $(HOMEDIR)/common.mk

SOURCES = ${wildcard *.cc}
#OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
KIN_OBJECTS = ${KINEMATDIR}/irp6_postument/kinematic_model_irp6p_with_wrist.o \
	      ${KINEMATDIR}/irp6_postument/kinematic_model_irp6p_5dof.o \
	      ${KINEMATDIR}/irp6_postument/kinematic_model_calibrated_irp6p_with_wrist.o \
	      ${KINEMATDIR}/irp6_postument/kinematic_model_calibrated_correction_matrix_irp6p_with_wrist.o \
	      ${KINEMATDIR}/irp6_postument/kinematic_model_irp6p_jacobian_with_wrist.o \
	      ${KINEMATDIR}/irp6_postument/kinematic_model_irp6p_jacobian_transpose_with_wrist.o
	       
OBJECTS = ../common/edp_vsp_t.o edp_irp6p_effector.o hi_irp6p.o int_hand_irp6p.o\
           regulator_irp6p.o sg_irp6p.o ../common/edp_irp6s_postument_track.o
EDPOBJS  = ${OBJECTS} ${KIN_OBJECTS}
BINARIES  = edp_irp6_postument


SHOBJ = ../common/libedp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
# do wyboru wariant z czujnikiem ATI3084 i ATI6284
ifneq ($(BUILD_TARGET), linux)
edp_irp6_postument: $(EDPOBJS) ../ati6284/ati6284.a $(SHOBJ)
#edp_irp6_postument: $(EDPOBJS) ../ati3084/ati3084.a $(SHOBJ)
else
edp_irp6_postument: $(EDPOBJS) $(SHOBJ)
endif
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
	@rm -f *.o ${BINARIES} ./.deps/*.P
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

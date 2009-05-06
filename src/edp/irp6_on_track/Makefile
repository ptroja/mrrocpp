#--------------- edp/irp6_on_track catalog makefile of the MRROC++ frame system ----------------#
#======================================================#

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
KINEMATDIR = ${HOMEDIR}/src/kinematics
include $(HOMEDIR)/common.mk

SOURCES = ${wildcard *.cc}
KIN_OBJECTS = ${KINEMATDIR}/irp6_on_track/kinematic_model_irp6ot_with_track.o \
	      ${KINEMATDIR}/irp6_on_track/kinematic_model_irp6ot_with_wrist.o \
	      ${KINEMATDIR}/irp6_on_track/kinematic_model_calibrated_irp6ot_with_wrist.o \
	      ${KINEMATDIR}/irp6_on_track/kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist.o

OBJECTS = force.o edp_irp6ot_effector.o hi_local.o int_hand.o sg_local.o edp_irp6s_postument_track.o
EDPOBJS  = ${OBJECTS} ${KIN_OBJECTS}
BINARIES  = edp_irp6_on_track

SHOBJ = ../common/libedp.a $(LIBDIR)/libmrrocpp.a
# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = ../common $(LIBDIR) ../irp6s
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
# do wyboru wariant z czujnikiem ATI3084 i ATI6284
#edp_irp6_on_track: $(EDPOBJS) ../ati6284/ati6284.a $(SHOBJ)
edp_irp6_on_track: $(EDPOBJS) ../ati3084/ati3084.a $(SHOBJ)
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
	@rm -f *.o ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

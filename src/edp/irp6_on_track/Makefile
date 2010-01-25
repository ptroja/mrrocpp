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
	      ${KINEMATDIR}/irp6_on_track/kinematic_model_calibrated_irp6ot_with_wrist.o $(KINEMATDIR)/common/libkinematics.a

OBJECTS = ../common/edp_vsp_t.o edp_irp6ot_effector.o hi_irp6ot.o\
 int_hand_irp6ot.o sg_irp6ot.o regulator_irp6ot.o ../common/edp_irp6s_postument_track.o
EDPOBJS = ${OBJECTS} ${KIN_OBJECTS}
BINARIES = edp_irp6_on_track

SHOBJ = ../common/libedp.a $(LIBDIR)/libmrrocpp.a

# add required libs
ifdef DOCENT_SENSOR
LDFLAGS:=$(LDFLAGS) -lboost_system-mt -lkiper -lprotobuf
endif

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
# do wyboru wariant z czujnikiem ATI3084 i ATI6284
#edp_irp6_on_track: $(EDPOBJS) ../ati6284/ati6284.a $(SHOBJ)

SENSOR_OBJ=
ifneq ($(BUILD_TARGET), linux)
ifdef DOCENT_SENSOR
	SENSOR_OBJ := ../ati3084MS/ati3084MS.a
else
	SENSOR_OBJ := ../ati3084/ati3084.a
endif
endif
edp_irp6_on_track: $(EDPOBJS) $(SENSOR_OBJ) $(SHOBJ)
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

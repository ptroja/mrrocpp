#------------- edp/irp6_postument catalog makefile of the MRROC++ frame system ---------------#
#======================================================#

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SOURCES = ${wildcard *.cc}
#OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
KIN_OBJECTS = ${KIN_DIR}/irp6_postument/libkinematicsirp6p.a $(KIN_DIR)/common/libkinematics.a
	       
OBJECTS = ../common/edp_vsp_t.o edp_irp6p_m_effector.o hi_irp6p_m.o int_hand_irp6p_m.o\
           ../common/regulator_irp6p_m.o sg_irp6p_m.o
EDPOBJS  = ${OBJECTS} ${KIN_OBJECTS}
BINARIES  = edp_irp6p_m


SHOBJ = ../common/libedp.a $(LIBDIR)/libmrrocpp.a

ifdef DOCENT_SENSOR
LDFLAGS:=$(LDFLAGS) -lboost_system-mt-1_39 -lkiper -lprotobuf
endif
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
# do wyboru wariant z czujnikiem ATI3084 i ATI6284, ati6284KT
ifneq ($(BUILD_TARGET), linux)
#edp_irp6p_m: $(EDPOBJS) ../ati6284KT/ati6284KT.a $(SHOBJ)
ifdef DOCENT_SENSOR
edp_irp6p_m: $(EDPOBJS) ../ati6284MS/ati6284MS.a $(SHOBJ)
else
edp_irp6p_m: $(EDPOBJS) ../ati6284/ati6284.a $(SHOBJ)
endif
#edp_irp6p_m: $(EDPOBJS) ../ati3084/ati3084.a $(SHOBJ)
else
edp_irp6p_m: $(EDPOBJS) $(SHOBJ)
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

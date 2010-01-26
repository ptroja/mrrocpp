#------------- edp/irp6_postument catalog makefile of the MRROC++ frame system ---------------#
#======================================================#

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SOURCES = ${wildcard *.cc}
#OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
KIN_OBJECTS = ${KIN_DIR}/spkm/libkinematicsspkm.a $(KIN_DIR)/common/libkinematics.a

OBJECTS = edp_e_spkm.o
EDPOBJS  = ${OBJECTS} ${KIN_OBJECTS}
BINARIES  = edp_spkm

SHOBJ = ../common/int_hand_suplement.o ../common/libedp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
edp_spkm: $(EDPOBJS) $(SHOBJ)
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

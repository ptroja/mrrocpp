#------------------ vsp/schunk catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Tworzy	: DIGITAL SCALES VSP for QNX 6.3.1 (interaktywna wspolpraca z oczekiwaniem z linialami)
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
VSPOBJS = ../common/vsp_m_int.o $(OBJECTS) $(SHOBJ)
BINARIES = vsp_digital_scales_sensor
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libmrrocpp.a ../common/vsp_sensor.o
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
vsp_digital_scales_sensor: $(VSPOBJS)
	$(LD) $(LDFLAGS) -o $@  $^
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
	@rm -f *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

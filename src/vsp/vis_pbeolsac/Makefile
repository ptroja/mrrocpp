#------------------ vsp/schunk catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Tworzy	: SHUNK VSP for QNX 6.2.1 (nieinteraktywna wspolpraca z czujnikiem sily)
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
VSPOBJS = ../common/vsp_m_int_nw.o $(OBJECTS) $(SHOBJ) 
BINARIES = vsp_vis_pbeolsac
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libmrrocpp.a ../common/vsp_sensor.o ../vis/cmvision.o \
	../vis/cube.o ../vis/global.o ../vis/calib.o ../vis/macierze_nr.o
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
vsp_vis_pbeolsac: $(VSPOBJS)
	$(LD) $(LDFLAGS) -o $@  $^
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	cp ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

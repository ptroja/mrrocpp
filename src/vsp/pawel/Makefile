#------------------ vsp/schunk catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Tworzy	: PAWEL VSP for QNX 6.2.1 
SOURCES = vsp_pawel.cc
OBJECTS = vsp_pawel.o
VSPOBJS = ../common/vsp_sensor.o ../common/vsp_m_int_nw.o $(OBJECTS) $(SHOBJ) 
BINARIES = vsp_pawel
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
vsp_pawel: $(VSPOBJS)
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

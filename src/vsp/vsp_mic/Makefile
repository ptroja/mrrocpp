#------------------ vsp/schunk catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Tworzy	: SHUNK VSP for QNX 6.2.1 (nieinteraktywna wspolpraca z czujnikiem sily)
SOURCES = vsp_mic.cc
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
VSPOBJS = ../common/vsp_m_int_nw.o $(OBJECTS) $(SHOBJ) 
BINARIES = vsp_mic
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libmrrocpp.a vsp_mic.o ../common/vsp_sensor.o \
	$(QNX_TARGET)/x86/lib/libasound.so
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
vsp_mic: $(VSPOBJS)
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
depend:
	@touch .depend
	@makedepend	-f.depend -I$(HOMEDIR)/include  \
							${patsubst %,-I%,${SYS_HEADER_DIRS}} \
							${SOURCES} 2> /dev/null

-include .depend

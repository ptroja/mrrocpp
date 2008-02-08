#------------------ vsp/vsp_rcs_kociemba catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Tworzy	: RCS Kociemba VSP for QNX 6.2.1 (nieinteraktywna wspolpraca z algorytmem Kociemby)

RCS_SRCS=${wildcard ../rcs/General/*.cc} ${wildcard ../rcs/Combinatorials/*.cc} ${wildcard ../rcs/Cube/*.cc} ${wildcard ../rcs/Kociemba/*.cc} ${wildcard ../rcs/Korf/*.cc}
RCS_OBJS = ${patsubst %.cc,%.o,${RCS_SRCS}}
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
VSPOBJS = ../common/vsp_m_nint.o ../common/vsp_sensor.o $(OBJECTS) $(SHOBJ) ${RCS_OBJS}
BINARIES = vsp_rcs_kociemba
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
vsp_rcs_kociemba: $(VSPOBJS)
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

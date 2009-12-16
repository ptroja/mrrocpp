#------------------ vsp/vsp_rcs_korf catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Tworzy	: RCS Korf VSP for QNX 6.2.1 (interaktywna wspolpraca bez oczekiwania z algorytmem Korfa)
RCS_SRCS=${wildcard ../rcs/General/*.cc} ${wildcard ../rcs/Combinatorials/*.cc} ${wildcard ../rcs/Cube/*.cc} ${wildcard ../rcs/Kociemba/*.cc} ${wildcard ../rcs/Korf/*.cc}
RCS_OBJS = ${patsubst %.cc,%.o,${RCS_SRCS}}
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
VSPOBJS = ../common/vsp_m_int_nw.o $(OBJECTS) $(SHOBJ) ${RCS_OBJS}
BINARIES = vsp_rcs_korf
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libmrrocpp.a ../common/vsp_sensor.o
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
vsp_rcs_korf: $(VSPOBJS)
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
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

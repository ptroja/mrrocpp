#------------------ vsp/vsp_rcs_korf catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Tworzy	: RCS Korf VSP for QNX 6.2.1 (interaktywna wspolpraca bez oczekiwania z algorytmem Korfa)
RCS_SRCS=${wildcard ../rcs/General/*.cc} ${wildcard ../rcs/Combinatorials/*.cc} ${wildcard ../rcs/Cube/*.cc} ${wildcard ../rcs/Kociemba/*.cc} ${wildcard ../rcs/Korf/*.cc}
RCS_OBJS = ${patsubst %.cc,%.o,${RCS_SRCS}}
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
VSPOBJS = vsp_m_int_nw.o $(OBJECTS) $(SHOBJ) ${RCS_OBJS}
BINARIES = vsp_rcs_korf
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libmrrocpp.a vsp_sensor.o
# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = $(LIBDIR) ../common
#======================================================#	
.PHONY: all clean install depend
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
	@rm -f *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

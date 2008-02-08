#------------------ vsp/rcs catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Tworzy	: RCS - rozwiazywanie kostki Rubika

SOURCES=${wildcard ../rcs/General/*.cc} ${wildcard ../rcs/Combinatorials/*.cc} ${wildcard ../rcs/Cube/*.cc} ${wildcard ../rcs/Kociemba/*.cc} ${wildcard ../rcs/Korf/*.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libmrrocpp.a vsp_sensor.o
# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = $(LIBDIR) ../common
#======================================================#	
.PHONY: all clean depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${OBJECTS}
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f ${OBJECTS}
#-----------------------------------------------------------------------------------------------------------------------#
install:
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

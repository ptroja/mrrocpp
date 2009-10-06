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
#======================================================#	
.PHONY: all clean
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${OBJECTS}
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f ${OBJECTS}
#-----------------------------------------------------------------------------------------------------------------------#
install:
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

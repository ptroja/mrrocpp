SOURCES = cmvision.cc calib.cc global.cc macierze_nr.cc cube.cc
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${OBJECTS}
#----------------------------------------------- Maintain rules ----------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

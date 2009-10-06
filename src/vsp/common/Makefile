#---------------- vsp/common catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
ifeq ($(BUILD_TARGET), linux)
  SOURCES = vsp_sensor.cc vsp_m_int_srr.cc
else
  SOURCES = ${wildcard *.cc}
endif
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
#BINARIES = 
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${OBJECTS}
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#vsp_m_int_nw.o:  vsp_m_int_nw.cc
#	$(CXX) $(FLAGS) -I$(HOMEDIR)/include   -c -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
#install:
#	@${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err 

#@$(MAKE) -C clean
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

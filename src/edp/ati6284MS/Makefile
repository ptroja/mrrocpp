#--------------- edp/irp6_on_track catalog makefile of the MRROC++ frame system ----------------#
#======================================================#
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
EDPOBJS  = ${OBJECTS}
#edp_common.o trans_t.o edp_irp6s.o 
#BINARIES  = ati6284KT 

LIBS = ati6284MS.a
#  -lsocket

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${LIBS}
#-----------------------------------------------------------------------------------------------------------------------#

edp_s.o: edp_s.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -lsocket -c -o $@ $<


ati6284MS.a: $(EDPOBJS)
	$(AR) $(ARFLAGS) $@ $^
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#


	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install:
	
#	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o ${LIBS}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

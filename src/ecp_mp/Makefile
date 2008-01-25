#-----------ecp_mp/common catalog makefile of the MRROC++ frame system ---------------------#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${OBJECTS}

#-----------------------------------------------------------------------------------------------------------------------#


#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
# install:
#	cp ${BINARIES} $(BINDIR)
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

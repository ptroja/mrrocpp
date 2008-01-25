#---------------- vsp/common catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
#BINARIES = 
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

#======================================================#	
.PHONY: all clean install depend player
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${OBJECTS} player
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#vsp_m_int_nw.o:  vsp_m_int_nw.cc
#	$(CXX) $(FLAGS) -I$(HOMEDIR)/include   -c -o $@ $<

player:
	@$(MAKE) -C $@
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
#install:
#	@cp ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err 
	@$(MAKE) -C player clean
#-----------------------------------------------------------------------------------------------------------------------#
depend:
	@touch .depend
	@makedepend	-f.depend -I$(HOMEDIR)/include  \
							${patsubst %,-I%,${SYS_HEADER_DIRS}} \
							${SOURCES} 2> /dev/null

-include .depend

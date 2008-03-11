#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES = ecp_speechrecognition ecp_playerpos
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = $(LIBDIR)/libecp.a $(LIBDIR)/libmrrocpp.a $(LIBDIR)/libplayerclient.a
# path for SHOBJ
# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = ../common $(ECP_MPDIR)
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#---------------------------------Special object build rules (user made)-------------------------------------#

ecp_speechrecognition: ecp_t_speechrecognition.o ecp_g_speechrecognition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

ecp_playerpos: ecp_t_playerpos.o ecp_g_playerpos.o playerpos_goal.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#-----------------------------------------------------------------------------------------------------------------------#
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

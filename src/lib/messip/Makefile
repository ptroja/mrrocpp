#---------------------- lib catalog makefile of the MRROC++ frame system -----------------------------#
#======================================================#
SOURCES = ${wildcard *.c}
OBJECTS = ${patsubst %.c,%.o,${SOURCES}}
MESSIP_OBJECTS = logg_messip.o messip_lib.o messip_utils.o
BINARIES = messip_sin messip_mgr
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

CPPFLAGS += -I$(HOMEDIR)/include/messip

#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: libmessip.a ${BINARIES}
	@cp libmessip.a ${LIBDIR}

libmessip.a: ${MESSIP_OBJECTS}
	$(AR) $(ARFLAGS) $@ $?

messip_mgr: messip_mgr.o logg_messip.o libmessip.a

messip_sin: messip_sin.o libmessip.a

#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	cp ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES} libmessip.a
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

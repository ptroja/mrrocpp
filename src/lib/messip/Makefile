#---------------------- lib catalog makefile of the MRROC++ frame system -----------------------------#
#======================================================#
SOURCES = ${wildcard *.c} messip_dataport.cc
OBJECTS = ${patsubst %.c,%.o,${SOURCES}}
MESSIP_OBJECTS = logg_messip.o messip_lib.o messip_utils.o messip_dataport.o
BINARIES = messip_sin messip_mgr
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: libmessip.a libmessip.so ${BINARIES}
	@cp libmessip.a ${LIBDIR}

libmessip.a: ${MESSIP_OBJECTS}
	$(AR) $(ARFLAGS) $@ $?

libmessip.so: ${MESSIP_OBJECTS}
	$(LD) $^ $(LDFLAGS) -shared -o $@
	${INSTALL} $@ $(LIBDIR)

messip_mgr: messip_mgr.o logg_messip.o libmessip.a

messip_sin: messip_sin.o libmessip.a

# for testing 
client: client.o libmessip.a
server: server.o libmessip.a

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
	@rm -f *.o *.err ${BINARIES} *.a *.so server client
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

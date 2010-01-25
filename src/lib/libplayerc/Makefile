#---------------------- lib catalog makefile of the MRROC++ frame system -----------------------------#
#======================================================#
SOURCES = ${wildcard *.c}
OBJECTS = ${patsubst %.c,%.o,${SOURCES}}
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

CPPFLAGS += -DHAVE_CONFIG_H -I$(HOMEDIR)/src/player
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: libplayerc.a

libplayerc.a: ${OBJECTS}
	$(AR) $(ARFLAGS) $@ $?
	@${INSTALL} $@ $(LIBDIR)
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
#install:
#	${INSTALL} ${OBJECTS} $(LIBDIR)
#	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P libplayerc.a
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

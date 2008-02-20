#---------------------- lib catalog makefile of the MRROC++ frame system -----------------------------#
#======================================================#
SOURCES = ${wildcard *.cc}  ${wildcard *.c}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}} ${patsubst %.c,%.o,${SOURCES}}
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

LIBPLAYERCLIENT=libplayerclient.a

CPPFLAGS += -I$(HOMEDIR)/include -DHAVE_CONFIG_H -I$(HOMEDIR)/include/player
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(LIBPLAYERCLIENT)

$(LIBPLAYERCLIENT): ${OBJECTS}
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
	@rm -f *.o *.a *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

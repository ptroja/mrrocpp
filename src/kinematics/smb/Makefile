#-----------ecp_mp/common catalog makefile of the MRROC++ frame system ---------------------#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: libkinematicssmb.a libkinematicssmb.so

libkinematicssmb.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $?

ifeq ($(BUILD_TARGET), linux)
LDFLAGS+=-Wl,-export-dynamic
endif

libkinematicssmb.so: $(OBJECTS)
	$(LD) $^ $(LDFLAGS) -shared -o $@
	${INSTALL} $@ $(LIBDIR)

#-----------------------------------------------------------------------------------------------------------------------#


#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
# install:
#	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.a *.o *.err ${BINARIES} ./.deps/*.P
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

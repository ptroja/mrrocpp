SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

LIBPNEXEC = libpnexec.a

# for XSD binder
CPPFLAGS+=-I/usr/local/include
LDFLAGS+=-lxerces-c

%.cxx: %.xsd
	xsd cxx-tree --type-regex '/ (.+)/$$1_t/' --root-element pnml $<

%.o : %.cxx
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../../..
include $(HOMEDIR)/common.mk

#======================================================#	
.PHONY: all clean xsd_and_lib_target
#------------------------------------------------- Build rules -------------------------------------------------------#
all: pipepnml.cxx $(LIBPNEXEC)

$(OBJECTS): pipepnml.cxx

$(LIBPNEXEC): pipepnml.o $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $?

#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -rf *.o *.?xx pipepnml.cc
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

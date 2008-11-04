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
#	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

#include $(HOMEDIR)/depend.mk

## Compilation scripts #################################################################################
# Directory with dependences.
DEPDIR = .deps
df = $(DEPDIR)/$(*F)
# Makedepend command.
MAKEDEPEND = mkdir -p .deps; gcc -M $(CXXFLAGS) $(CPPFLAGS) -o $(DEPDIR)/$*.d $<
#MAKEDEPEND_QCC = mkdir -p .deps; QCC -M $(CXXFLAGS) $(CPPFLAGS) -o $(DEPDIR)/$* $<

# Overwritten command for *.cc compilation - also creates dependency files (*.P) in adequate directory.
%.o : %.cc
	@$(MAKEDEPEND); \
#	@$(MAKEDEPEND_QCC); \
	cp $(df).d $(df).P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $(df).d >> $(df).P; \
#	rm -f $(df).d
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

# Wildcards used during dependency include.
#SOURCES = ${wildcard *.cc}
DEPENDS = ${patsubst %.cc,$(DEPDIR)/%.P,${SOURCES}}

# Include created dependency files.
-include $(DEPENDS)



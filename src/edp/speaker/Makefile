#------------- edp/irp6_postument catalog makefile of the MRROC++ frame system ---------------#
#======================================================#
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
EDPOBJS  = ${OBJECTS}
BINARIES  = edp_speaker
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

CXXFLAGS += -O3
SHOBJ = ../common/libedp.a $(LIBDIR)/libmrrocpp.a $(QNX_TARGET)/x86/lib/libasound.so
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
edp_speaker: $(EDPOBJS) $(SHOBJ) 
	$(LD) $(LDFLAGS) -o $@ $^ 
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
	@rm -f *.o ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

#--------------- edp/irp6_on_track catalog makefile of the MRROC++ frame system ----------------#
#======================================================#
SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
EDPOBJS  = ${OBJECTS}
#edp_common.o trans_t.o edp_irp6s.o 
#BINARIES  = ati6284 

LIBS = ati6284.a

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

CPPFLAGS += -I$(HOMEDIR)/src/edp/ati6284/ati \
		-I$(HOMEDIR)/src/edp/ati6284/ni \
		-I$(HOMEDIR)/src/edp/ati6284 \
		-I$(HOMEDIR)/src/edp/ati6284
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${LIBS}
#-----------------------------------------------------------------------------------------------------------------------#
ati6284.a: $(EDPOBJS)
	$(AR) $(ARFLAGS) $@ $^ 
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install:
	
#	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o ${LIBS}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

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

CPPFLAGS += -I$(HOMEDIR)/include/edp/ati6284/ati \
		-I$(HOMEDIR)/include/edp/ati6284/ni \
		-I$(HOMEDIR)/include/edp/ati6284 \
		-I$(HOMEDIR)/src/edp/ati6284

# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = ../common $(LIBDIR) ../irp6s
#======================================================#	
.PHONY: all clean install depend
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
	
#	cp ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o ${LIBS}
#-----------------------------------------------------------------------------------------------------------------------#
depend:
	@touch .depend
	@makedepend	-f.depend -I$(HOMEDIR)/include  \
							${patsubst %,-I%,${SYS_HEADER_DIRS}} \
							${SOURCES} 2> /dev/null

-include .depend

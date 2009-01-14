#--------------- edp/conveyor catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
KINEMATDIR = ${HOMEDIR}/src/kinematics
include $(HOMEDIR)/common.mk

KIN_OBJECTS = ${KINEMATDIR}/conveyor/kinematic_model_conveyor.o

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES = edp_conveyor

SHOBJ = ../common/libedp.a $(LIBDIR)/libmrrocpp.a
# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = ../common $(LIBDIR) ../irp6s
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
edp_conveyor: $(OBJECTS) ${KIN_OBJECTS} $(SHOBJ)
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

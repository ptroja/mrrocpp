#------------- edp/irp6_postument catalog makefile of the MRROC++ frame system ---------------#
#======================================================#
SOURCES = ${wildcard *.cc}
#OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
KIN_OBJECTS = kinematic_model_irp6m_with_wrist.o \
	      kinematic_model_irp6m_5dof.o

OBJECTS = hi_local.o int_hand.o sg_local.o edp_irp6m_effector.o
EDPOBJS  = ${OBJECTS} ${KIN_OBJECTS}
BINARIES  = edp_irp6_mechatronika
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ../common/libedp.a $(LIBDIR)/libmrrocpp.a
# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = ../common $(LIBDIR) ../irp6s
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
# do wyboru wariant z czujnikiem ATI3084 i ATI6284
#edp_irp6_mechatronika: $(EDPOBJS) ../ati3084/ati3084.a $(SHOBJ)
edp_irp6_mechatronika: $(EDPOBJS) $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	cp ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
depend:
	@touch .depend
	@makedepend	-f.depend -I$(HOMEDIR)/include  \
							${patsubst %,-I%,${SYS_HEADER_DIRS}} \
							${SOURCES} 2> /dev/null

-include .depend

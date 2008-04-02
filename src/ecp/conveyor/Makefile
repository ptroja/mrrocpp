#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES = 	ecp_test_conv ecp_kon ecp_legobrick
				
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ecp_local.o ../common/libecp.a $(LIBDIR)/libmrrocpp.a
# path for SHOBJ
# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = ../common $(ECP_MPDIR)
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#---------------------------------Special object build rules (user made)-------------------------------------#

ecp_test_conv: ecp_t_test.o ecp_g_test.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_kon: ecp_t_kon.o ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
ecp_legobrick: ecp_t_legobrick_conv.o ecp_g_test.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
#-----------------------------------------------------------------------------------------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

#-------- ecp_mp/irp6_postument catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES = 	ecp_test_irp6p \
				ecp_jarosz_irp6p\
				ecp_irp6p_rcsc\
				ecp_dung\
				ecp_irp6p_pouring

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

ecp_test_irp6p: ecp_t_test_irp6p.o ecp_g_force_pose_test.o  ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_irp6p_rcsc: ecp_t_rcsc_irp6p.o ecp_g_smooth.o ecp_g_force.o  ecp_t_rcsc.o ecp_mp_s_schunk.o ecp_generator_t.o \
		ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
	
#-----------------------------------------------------------------------------------------------------------------------#
ecp_jarosz_irp6p: ecp_t_jarosz_irp6p.o ecp_g_jarosz.o ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@  $^

ecp_dung: ecp_t_dung.o ecp_g_dung.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@  $^

ecp_irp6p_pouring: ecp_t_pouring_irp6p.o ecp_t_pouring.o ecp_g_smooth.o ecp_generator_t.o ecp_g_jarosz.o \
	ecp_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
	
#-----------------------------------------------------------------------------------------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: ${BINARIES}
	cp ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:

	@rm -f *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
depend:
	@touch .depend
	@makedepend	-f.depend -I$(HOMEDIR)/include  \
							${patsubst %,-I%,${SYS_HEADER_DIRS}} \
							${SOURCES} 2> /dev/null

-include .depend

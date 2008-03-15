#-----------ecp_mp/common catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES = ecp_tran ecp_mam ecp_t ecp_nalewanie ecp_sk ecp_time ecp_pawel

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ../common/libecp.a $(LIBDIR)/libmrrocpp.a

# search path for make, if you want create dependencies from files from other directory than ".", 
# then place that directory in the VPATH variable
VPATH = $(ECP_MPDIR)
#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}

#-----------------------------------------------------------------------------------------------------------------------#

ecp_tran: \
		../common/ecp_t_tran.o \
		../irp6_on_track/ecp_local.o \
		../irp6_postument/ecp_local.o \
		../conveyor/ecp_local.o \
		../speaker/ecp_local.o \
		../irp6_mechatronika/ecp_local.o \
		../common/ecp_generator_t.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_mam: \
		../irp6_on_track/ecp_local.o \
		../irp6_postument/ecp_local.o \
		../common/ecp_t_mam.o \
		../common/ecp_g_mam.o \
		ecp_mp_s_digital_scales.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_sk: \
		../irp6_on_track/ecp_local.o \
		../irp6_postument/ecp_local.o \
		../common/ecp_t_sk.o \
		../common/ecp_g_force.o \
		ecp_mp_s_schunk.o \
		ecp_mp_s_cvfradia.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_t: \
		../irp6_on_track/ecp_local.o \
		../irp6_postument/ecp_local.o \
		../irp6_mechatronika/ecp_local.o  \
		../common/ecp_t_teach.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_nalewanie: \
		../irp6_on_track/ecp_local.o \
		../irp6_postument/ecp_local.o \
		../common/ecp_t_nalewanie.o \
		../common/ecp_g_smooth.o \
		../common/ecp_g_jarosz.o \
		../common/ecp_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_time: \
		../irp6_on_track/ecp_local.o \
		../irp6_postument/ecp_local.o \
		ecp_mp_s_time.o \
		../common/ecp_t_time.o \
		../common/ecp_g_time.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

ecp_pawel: \
		../irp6_on_track/ecp_local.o \
		../irp6_postument/ecp_local.o \
		../common/ecp_t_pawel.o \
		../common/ecp_g_pawel.o \
		ecp_mp_s_pawel.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 


#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
install: $(BINARIES)
	${INSTALL} ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.a *.o *.err ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

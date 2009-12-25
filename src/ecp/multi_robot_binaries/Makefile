#-----------ecp_mp/common catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES = ecp_tran \
#	ecp_visioncoordinates \
	ecp_fsautomat

	

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ../common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}

#-----------------------------------------------------------------------------------------------------------------------#

ecp_tran: \
		../common/task/ecp_t_tran.o \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../conveyor/ecp_r_conv.o \
		../speaker/ecp_r_speaker.o \
		../irp6_mechatronika/ecp_r_irp6m.o \
		../polycrank/ecp_r_polycrank.o \
		../common/generator/ecp_g_transparent.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 


ecp_fsautomat: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/task/ecp_t_fsautomat.o \
		../../ecp_mp/Trajectory.o \
		../common/generator/ecp_g_force.o \
		../common/task/ecp_st_go.o \
		../common/generator/ecp_g_smooth.o \
		../common/generator/ecp_g_transparent.o \
		../common/generator/ecp_g_jarosz.o \
		../common/generator/ecp_g_operator_reaction_condition.o \
		$(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^
	

ecp_visioncoordinates: \
		../irp6_on_track/ecp_r_irp6ot.o \
		../irp6_postument/ecp_r_irp6p.o \
		../common/task/ecp_t_visioncoordinates.o \
		../common/generator/ecp_g_visioncoordinates.o \
		../common/generator/ecp_g_smooth.o \
		../common/generator/ecp_g_jarosz.o \
		../common/generator/ecp_g_operator_reaction_condition.o \
		$(ECP_MPDIR)/sensor/ecp_mp_s_cvfradia.o \
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
	@rm -f *.a *.o *.err ${BINARIES} ./.deps/*.P ./task/*.o ./task/*.err ./generator/*.o ./generator/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

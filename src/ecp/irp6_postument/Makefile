#-------- ecp_mp/irp6_postument catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}
BINARIES = 			ecp_jarosz_irp6p\
				ecp_irp6p_rcsc\
				ecp_dung\
				#ecp_irp6p_pouring\
				#ecp_legobrick_irp6p\
				ecp_test_irp6p

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ecp_r_irp6p.o ../common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#---------------------------------Special object build rules (user made)-------------------------------------#

ecp_test_irp6p: task/ecp_t_test_irp6p.o generator/ecp_g_force_pose_test.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 

#-----------------------------------------------------------------------------------------------------------------------#
ecp_irp6p_rcsc: task/ecp_t_rcsc_irp6p.o ../common/generator/ecp_g_smooth2.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o ../common/generator/ecp_g_t.o \
		../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^ 
	
#-----------------------------------------------------------------------------------------------------------------------#
#ecp_legobrick_irp6p: task/ecp_t_legobrick_irp6p.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_force.o ../common/task/ecp_st_go.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o ../common/generator/ecp_g_t.o \
#		../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^ 

#-----------------------------------------------------------------------------------------------------------------------#
ecp_jarosz_irp6p: task/ecp_t_jarosz_irp6p.o ../common/generator/ecp_g_jarosz.o ../common/generator/ecp_g_operator_reaction_condition.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

ecp_dung: task/ecp_t_dung.o generator/ecp_g_dung.o $(SHOBJ)
	$(LD) $(LDFLAGS) -o $@ $^

#ecp_irp6p_pouring: task/ecp_t_pouring_irp6p.o ../common/task/ecp_st_go.o ../common/generator/ecp_g_smooth.o ../common/generator/ecp_g_t.o ../common/generator/ecp_g_jarosz.o \
#	../common/generator/ecp_g_operator_reaction_condition.o $(ECP_MPDIR)/sensor/ecp_mp_s_schunk.o $(SHOBJ)
#	$(LD) $(LDFLAGS) -o $@ $^ 
		

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

	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ./task/*.o ./task/*.err ./generator/*.o ./generator/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}


BINARIES =	 ecp_draughts 

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif

#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

SHOBJ = ../../ecp/irp6_on_track/ecp_r_irp6ot.o ../../ecp/irp6_postument/ecp_r_irp6p.o ../../ecp/common/libecp.a $(LIBDIR)/libmrrocpp.a
#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}



ecp_draughts: ecp_t_draughts.o ecp_mp_tr_draughtsAI.o ecp_g_vis_ib_eih_follower_irp6ot.o ../../ecp/irp6_on_track/generator/ecp_g_vis_ib_eih_planar_irp6ot.o \
../../ecp/common/generator/ecp_g_force.o ../../ecp/common/generator/ecp_g_transparent.o ../../ecp/common/task/ecp_st_go.o \
../../ecp/common/generator/ecp_g_sleep.o \
../../ecp_mp/sensor/ecp_mp_s_cvfradia.o  $(SHOBJ) 
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
	@rm -f *.o *.err ${BINARIES} ./.deps/*.P ./sensor/*.o ./sensor/*.err ./generator/*.o ./generator/*.err
#-----------------------------------------------------------------------------------------------------------------------#

include $(HOMEDIR)/depend.mk

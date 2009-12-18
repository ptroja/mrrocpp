#----------- ecp_mp/irp6_on_track catalog makefile of the MRROC++ frame system --------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES = ecp_mboryn

ifneq ($(BUILD_TARGET), linux)
BINARIES+=			
endif


#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

#SHOBJ = ../../ecp_mp/libecp_mp.a ../../ecp_mp/ecp_mp_robot.o ../../mp/libmpmain.a ../../mp/libmprobots.a $(LIBDIR)/libmrrocpp.a
#SHOBJ = #ecp_r_irp6ot.o ../common/libecp.a $(LIBDIR)/libmrrocpp.a
SHOBJ = ../../ecp_mp/libecp_mp.a ../../ecp_mp/ecp_mp_robot.o ../../ecp/irp6_on_track/ecp_r_irp6ot.o ../../ecp/irp6_postument/ecp_r_irp6p.o ../../ecp/common/libecp.a $(LIBDIR)/libmrrocpp.a

#======================================================#	
.PHONY: all clean install
#------------------------------------------------- Build rules -------------------------------------------------------#
all: ${BINARIES}
#-----------------------------------------------------------------------------------------------------------------------#
ecp_mboryn: ecp_t_mboryn.o ecp_g_mboryn.o $(SHOBJ)
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

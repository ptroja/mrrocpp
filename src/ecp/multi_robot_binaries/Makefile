#-----------ecp_mp/common catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#

SOURCES = ${wildcard *.cc}
OBJECTS = ${patsubst %.cc,%.o,${SOURCES}}

BINARIES = ecp_tran 
	
#-----------------------------------------------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

KIN_OBJECTS = \
	${KIN_DIR}/irp6_on_track/libkinematicsirp6ot.a \
	${KIN_DIR}/irp6_postument/libkinematicsirp6p.a \
	$(KIN_DIR)/common/libkinematics.a
SHOBJ = ${KIN_OBJECTS} ../common/libecp.a $(LIBDIR)/libmrrocpp.a 
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

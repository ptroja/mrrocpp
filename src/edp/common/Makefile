#---------------- edp/common catalog makefile of the MRROC++ frame system ---------------------#
#======================================================#
# Should be modified only by developers not by users
HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

EDPOBJ = edp_common.o edp_m.o \
	edp_irp6s.o edp_irp6s_and_conv.o edp_irp6s_postument_track.o \
	edp_force_sensor.o force.o \
	servo_gr.o servo_ms.o \
	trans_t.o vis_server.o reader.o \
	kinematic_model.o kinematics_manager.o \
	hi_rydz.o sg_irp6p_and_conv.o

#======================================================#	
.PHONY: all clean install depend
#------------------------------------------------- Build rules -------------------------------------------------------#
all: libedp.a

libedp.a: $(EDPOBJ)
	$(AR) $(ARFLAGS) $@ $?
#---------------------------------Special object build rules (user made)-------------------------------------#
# e.g:
# plik.o: plik.cc plik1.h plik2.h ... itd
#	$(CXX) -c user_flags -I$(HOMEDIR)/include -o $@ $<
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
#install:
#	cp ${BINARIES} $(BINDIR)
#-----------------------------------------------------------------------------------------------------------------------#
clean:
	@rm -f *.o *.a *.err 
#-----------------------------------------------------------------------------------------------------------------------#
depend:
	@touch .depend
	@makedepend	-f.depend -I$(HOMEDIR)/include  \
							${patsubst %,-I%,${SYS_HEADER_DIRS}} \
							${SOURCES} 2> /dev/null

-include .depend

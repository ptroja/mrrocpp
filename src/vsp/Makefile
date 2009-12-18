#------------------------- vsp catalog makefile of the MRROC++ frame system ------------------------#
#======================================================#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

SOURCEDIRS = common vsp_time_sensor
ifneq ($(BUILD_TARGET), linux)
  SOURCEDIRS+=  vsp_force_sensor vsp_mic vsp_nn vsp_pp
endif

CLEANDIRS = ${SOURCEDIRS}
INSTALLDIRS = ${SOURCEDIRS}

# some sheared objects
SHOBJ = $(LIBDIR)/libmrrocpp.a vsp_sensor.o
#======================================================#	
.PHONY: all clean install $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS)

schunk vsp_digital_scales_sensor vsp_force_sensor vsp_time_sensor vsp_pp: common
vis_pbeoleih vis_pbeolsac vis_pbeclsac vis_sac vsp_nn vis_sac_lx : common vis
vsp_mic pawel: common
vsp_rcs_kociemba vsp_rcs_korf: common rcs

$(SOURCEDIRS):
	@$(MAKE) -C $@

#----------------------------------------------- Maintain rules ----------------------------------------------------#
clean:
	@for i in ${CLEANDIRS} ; do ($(MAKE) -C $$i clean) ; done
#-----------------------------------------------------------------------------------------------------------------------#
install:
	@for i in ${INSTALLDIRS} ; do ($(MAKE) -C $$i install) ; done
#======================================================#	

#------------------------- vsp catalog makefile of the MRROC++ frame system ------------------------#
#======================================================#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

SOURCEDIRS = common rcs schunk vsp_digital_scales_sensor vsp_force_sensor vsp_time_sensor vis vis_pbeoleih vis_pbeolsac vis_pbeclsac vis_sac vis_sac_lx vsp_mic vsp_nn  vsp_rcs_kociemba vsp_rcs_korf pawel vsp_pp

CLEANDIRS = ${SOURCEDIRS}
INSTALLDIRS = ${SOURCEDIRS}
# Data for makedepend 
DEPENDDIRS = ${SOURCEDIRS}

# some sheared objects
SHOBJ = $(LIBDIR)/libmrrocpp.a vsp_sensor.o
#======================================================#	
.PHONY: all clean install depend $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS) $(DEPENDDIRS)
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
	@for i in ${CLEANDIRS} ; \
		do ($(MAKE) -C $$i clean) ; \
	done
#-----------------------------------------------------------------------------------------------------------------------#
install:
	@for i in ${INSTALLDIRS} ; \
		do ($(MAKE) -C $$i install) ; \
	done
#-----------------------------------------------------------------------------------------------------------------------#
depend:
	@for i in ${DEPENDDIRS}; do \
		($(MAKE) -C $$i depend); \
	done
#======================================================#	

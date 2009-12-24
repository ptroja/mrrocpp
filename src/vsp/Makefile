#------------------------- vsp catalog makefile of the MRROC++ frame system ------------------------#
#======================================================#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

SOURCEDIRS = common
ifneq ($(BUILD_TARGET), linux)
  SOURCEDIRS+= 
endif

CLEANDIRS = ${SOURCEDIRS}
INSTALLDIRS = ${SOURCEDIRS}

# some sheared objects
SHOBJ = $(LIBDIR)/libmrrocpp.a vsp_sensor.o
#======================================================#	
.PHONY: all clean install $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS)

 vsp_force_sensor vsp_time_sensor vsp_pp: common


$(SOURCEDIRS):
	@$(MAKE) -C $@

#----------------------------------------------- Maintain rules ----------------------------------------------------#
clean:
	@for i in ${CLEANDIRS} ; do ($(MAKE) -C $$i clean) ; done
#-----------------------------------------------------------------------------------------------------------------------#
install:
	@for i in ${INSTALLDIRS} ; do ($(MAKE) -C $$i install) ; done
#======================================================#	

#------------------- ecp_mp catalog makefile of the MRROC++ frame system ------------------------#
#======================================================#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

ifeq ($(BUILD_TARGET), linux)
#MANIPULATOR_SOURCEDIRS = irp6_on_track irp6_postument irp6_mechatronika conveyor
MANIPULATOR_SOURCEDIRS = irp6_mechatronika #irp6_on_track irp6_postument conveyor
SOURCEDIRS = common $(MANIPULATOR_SOURCEDIRS)
INSTALLDIRS = 
else
SOURCEDIRS = irp6_on_track conveyor irp6_postument speaker irp6_mechatronika polycrank festival player multi_robot_binaries common
INSTALLDIRS = ${SOURCEDIRS}
endif
CLEANDIRS = ${SOURCEDIRS}
# Data for makedepend 
DEPENDDIRS = ${SOURCEDIRS}
#======================================================#
.PHONY: all clean install depend $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS) $(DEPENDDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#

all: $(SOURCEDIRS)

irp6_on_track: common
irp6_postument: common
irp6_mechatronika: common
polycrank: common
conveyor: common
speaker: common
festival: common
player: common
multi_robot_binaries: irp6_on_track conveyor irp6_postument speaker irp6_mechatronika

$(SOURCEDIRS):
	@$(MAKE) -C $@

#	for i in ${SOURCEDIRS} ; do \
#		($(MAKE) -C $$i) ; \
#	done
#======================================================#	
#----------------------------------------------- Maintain rules ----------------------------------------------------#
clean:
	@for i in ${CLEANDIRS} ; do ($(MAKE) -C $$i clean) ; done
#-----------------------------------------------------------------------------------------------------------------------#
install:
	@for i in ${INSTALLDIRS} ; do ($(MAKE) -C $$i install) ; done
#-----------------------------------------------------------------------------------------------------------------------#
depend:
	@for i in ${DEPENDDIRS}; do ($(MAKE) -C $$i depend); done
#======================================================#	

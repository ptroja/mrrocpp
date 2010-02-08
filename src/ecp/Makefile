#------------------- ecp_mp catalog makefile of the MRROC++ frame system ------------------------#
#======================================================#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

SOURCEDIRS = irp6p_m irp6ot_tfg irp6ot_m irp6_on_track conveyor irp6_postument speaker irp6_mechatronika polycrank festival player smb spkm shead multi_robot_binaries common 
INSTALLDIRS = ${SOURCEDIRS}
CLEANDIRS = ${SOURCEDIRS}
#======================================================#
.PHONY: all clean install $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#

all: $(SOURCEDIRS)

irp6_on_track: common
irp6ot_m: common
irp6ot_tfg: common
irp6_postument: common
irp6p_m: common
irp6_mechatronika: common
polycrank: common
conveyor: common
speaker: common
festival: common
player: common
smb: common
spkm: common
shead: common
multi_robot_binaries: irp6_on_track conveyor irp6_postument speaker irp6_mechatronika smb spkm

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
#======================================================#	

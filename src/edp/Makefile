#------------------------- edp catalog makefile of the MRROC++ frame system ------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

SOURCEDIRS = common ati3084 ati3084MK ati6284 conveyor irp6_on_track speaker\
	polycrank irp6_postument irp6_mechatronika

ifdef DOCENT_SENSOR
	SOURCEDIRS += ati3084MS
endif

CLEANDIRS = ${SOURCEDIRS}
INSTALLDIRS = ${SOURCEDIRS}
# Data for makedepend 
DEPENDDIRS = ${SOURCEDIRS}
#======================================================#	
.PHONY: all clean install depend $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS) $(DEPENDDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS)

ati3084: common
ati6284: common
conveyor: common

IRP6ONT_DEPS = common ati3084 ati3084MK ati6284
ifdef DOCENT_SENSOR
	IRP6ONT_DEPS:=$(IRP6ONT_DEPS) ati3084MS
endif

irp6_on_track: $(IRP6ONT_DEPS)

speaker: common

IRP6PST_DEPS = common ati3084 ati3084MK ati6284
ifdef DOCENT_SENSOR
	IRP6PST_DEPS:= $(IRP6ONT_DEPS) ati3084MS
endif

irp6_postument: $(IRP6ONT_DEPS)

irp6_mechatronika: common
polycrank: common

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

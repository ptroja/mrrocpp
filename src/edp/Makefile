#------------------------- edp catalog makefile of the MRROC++ frame system ------------------------#
# Should be modified only by developers not by users
HOMEDIR = ../..
include $(HOMEDIR)/common.mk

ifeq ($(BUILD_TARGET), linux)
SOURCEDIRS = common \
	conveyor irp6_on_track irp6_postument irp6_mechatronika \
	smb spkm
else
SOURCEDIRS = common \
	irp6p_tfg irp6ot_tfg irp6p_m irp6ot_m\
	ati3084 ati3084MK ati6284 ati6284KT\
	conveyor irp6_on_track irp6_postument irp6_mechatronika \
	speaker polycrank \
	smb spkm shead	
endif

ifdef DOCENT_SENSOR
	SOURCEDIRS += ati6284MS
endif

CLEANDIRS = ${SOURCEDIRS}
INSTALLDIRS = ${SOURCEDIRS}
#======================================================#	
.PHONY: all clean install $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS)

ati3084: common
ati6284: common
ati6284KT: common
ifdef DOCENT_SENSOR
	ati6284MS: common
endif
conveyor: common
smb: common
spkm: common
shead: common


IRP6ONT_DEPS = common ati3084 ati3084MK ati6284 ati6284KT
ifdef DOCENT_SENSOR
	IRP6ONT_DEPS:=$(IRP6ONT_DEPS) ati6284MS
endif

irp6_on_track: $(IRP6ONT_DEPS)

speaker: common

IRP6PST_DEPS = common ati3084 ati3084MK ati6284 ati6284KT
ifdef DOCENT_SENSOR
	IRP6PST_DEPS:= $(IRP6ONT_DEPS) ati6284MS
endif

irp6_postument: $(IRP6ONT_DEPS)

irp6ot_m: $(IRP6ONT_DEPS)
irp6ot_tfg: $(IRP6ONT_DEPS)
irp6p_m: $(IRP6ONT_DEPS)
irp6p_tfg: $(IRP6ONT_DEPS)

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
#======================================================#	

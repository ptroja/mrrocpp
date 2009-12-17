#--------------------- src catalog makefile of the MRROC++ frame system ----------------------------#
# Should be modified only by developers not by users
HOMEDIR = ..
SOURCEDIRS = lib kinematics edp ecp ecp_mp mp ui application
ifneq ($(BUILD_TARGET), linux)
SOURCEDIRS+= bt848 vsp ui_server
endif

CLEANDIRS = $(SOURCEDIRS)
INSTALLDIRS = ${CLEANDIRS}
#======================================================#	
.PHONY: all clean install $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS) $(UIDIR)
#-----------------------------------------------------------------------------------------------------------------------#

kinematics edp ecp_mp vsp: lib
edp ecp_mp vsp: kinematics
mp ecp task: ecp_mp
application: ecp mp vsp
ui: ecp
ui_server: ui

$(SOURCEDIRS):
	@$(MAKE) -C $@

#----------------------------------------------- Maintain rules ----------------------------------------------------#
clean:
	@for i in ${CLEANDIRS} ; do ($(MAKE) -C $$i clean) ; done
#-----------------------------------------------------------------------------------------------------------------------#
install:
	@for i in ${INSTALLDIRS} ; do ($(MAKE) -C $$i install) ; done
#======================================================#	


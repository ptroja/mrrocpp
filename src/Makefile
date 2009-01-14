#--------------------- src catalog makefile of the MRROC++ frame system ----------------------------#
# Should be modified only by developers not by users
HOMEDIR = ..
# Data for makedepend 
ifeq ($(BUILD_TARGET), linux)
  SOURCEDIRS = lib kinematics vsp edp ecp ecp_mp mp
else
  SOURCEDIRS = bt848 lib kinematics vsp edp ecp ecp_mp mp ui ui_server
endif
CLEANDIRS = ${SOURCEDIRS}
INSTALLDIRS = ${CLEANDIRS}
#======================================================#	
.PHONY: all clean install depend $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS) $(UIDIR)
#-----------------------------------------------------------------------------------------------------------------------#

kinematics edp ecp_mp vsp: lib
edp ecp_mp vsp: kinematics
mp ecp: ecp_mp
ui: ecp
ui_server: ui

$(SOURCEDIRS):
	@$(MAKE) -C $@

#----------------------------------------------- Maintain rules ----------------------------------------------------#
clean:
	@for i in ${CLEANDIRS} ; do \
		($(MAKE) -C $$i clean) ; \
	done
#-----------------------------------------------------------------------------------------------------------------------#
install:
	@for i in ${INSTALLDIRS} ; do \
		($(MAKE) -C $$i install) ; \
	done
#-----------------------------------------------------------------------------------------------------------------------#
depend:
	@for i in ${SOURCEDIRS}; do \
		($(MAKE) -C $$i depend); \
	done
#======================================================#	

#--------------------- src catalog makefile of the MRROC++ frame system ----------------------------#
# Should be modified only by developers not by users
HOMEDIR = ..
# Data for makedepend 
ifeq ($(TARGET), qnx)
	SOURCEDIRS = lib edp
endif
ifeq ($(TARGET), linux)
	SOURCEDIRS = ecp ui
	LIBDIR = lib
endif
ifeq ($(TARGET), linux_gtk)
	SOURCEDIRS = ui/gtk
endif
ifneq ($(TARGET), qnx)
ifneq ($(TARGET), linux)
ifneq ($(TARGET), linux_gtk)
	SOURCEDIRS = bt848 lib kinematics vsp edp ecp ecp_mp mp ui ui_server
endif
endif
endif
ifeq ($(TARGET), linux)
	CLEANDIRS = ${LIBDIR}
else
	CLEANDIRS = $(SOURCEDIRS)
endif	
INSTALLDIRS = ${CLEANDIRS}
#======================================================#	
.PHONY: all clean install depend $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS) $(LIBDIR) $(UIDIR)
#-----------------------------------------------------------------------------------------------------------------------#

kinematics edp ecp_mp vsp: lib
edp ecp_mp vsp: kinematics
mp ecp: ecp_mp
ui: ecp
ui_server: ui

ifeq ($(TARGET), qnx)
$(SOURCEDIRS):
	@$(MAKE) CXXFLAGS=-DUSE_MESSIP_SRR -C $@
endif
ifeq ($(TARGET), linux)
$(SOURCEDIRS):
	@$(MAKE) TARGET=linux -B -C $@
$(LIBDIR):
	@$(MAKE) BUILD_TARGET=linux -C $@
endif
ifeq ($(TARGET), linux_gtk)
$(SOURCEDIRS):
	@$(MAKE) BUILD_TARGET=linux -B -C $@
endif
ifneq ($(TARGET), qnx)
ifneq ($(TARGET), linux)
ifneq ($(TARGET), linux_gtk)
$(SOURCEDIRS):
	@$(MAKE) -C $@
endif
endif
endif

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

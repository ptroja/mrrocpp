#--------------------- src catalog makefile of the MRROC++ frame system ----------------------------#
# Should be modified only by developers not by users
HOMEDIR = ..
UIDIR = ui
# Data for makedepend 
DEPENDDIRS = lib vsp edp ecp ecp_mp mp
SOURCEDIRS = $(DEPENDDIRS) $(UIDIR)
CLEANDIRS = ${SOURCEDIRS}
INSTALLDIRS = ${CLEANDIRS}
#======================================================#	
.PHONY: all clean install depend $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS) $(DEPENDDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS) $(UIDIR)
#-----------------------------------------------------------------------------------------------------------------------#

edp ecp_mp vsp: lib
mp ecp: ecp_mp
ui: ecp

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
	@for i in ${DEPENDDIRS}; do \
		($(MAKE) -C $$i depend); \
	done
#======================================================#	

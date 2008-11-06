#--------------------- src catalog makefile of the MRROC++ frame system ----------------------------#
# Should be modified only by developers not by users
HOMEDIR = ..
# Data for makedepend 
SOURCEDIRS = bttvxa testbt848
CLEANDIRS = ${SOURCEDIRS}
INSTALLDIRS = ${CLEANDIRS}
#======================================================#	
.PHONY: all clean install depend $(SOURCEDIRS) $(CLEANDIRS) $(INSTALLDIRS)
#------------------------------------------------- Build rules -------------------------------------------------------#
all: $(SOURCEDIRS) $(UIDIR)
#-----------------------------------------------------------------------------------------------------------------------#


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
	@true
#======================================================#	

#///////////////////////////////////////////////////////////////////////////////
# /** @file     Makefile
#  *
#  *  Main makefile of the MRROC++ frame system.
#  *
#  *  @author   tkornuta
#  *  @date     2007-01-12
#  *  $URL$
#  *  $LastChangedRevision$
#  *  $LastChangedDate$  
#  *  $LastChangedBy$
#  *  
#  */
#///////////////////////////////////////////////////////////////////////////////
 
  
#------------------------------------------------------------------------------#
# Should be modified only by developers not by users
HOMEDIR = .
CLEANDIRS = src lib
CLEANEDFILES = ./bin/* ./msr/* ./logs/* ./html/*
INSTALLDIRS = src

# Data for makedepend 
DEPENDDIRS = src
#======================================================#	
.PHONY: all distclean clean install depend dist


#------------------------------- Build rules ----------------------------------#
all:
	$(MAKE) -C src
#------------------------------------------------------------------------------#
#probably not nessery, should be removed in the future
#app:
#	for i in ${DIRS} ; do (cd $$i && $(MAKE) app) ; done
#
#proto:
#	for i in ${DIRS} ; do (cd $$i && $(MAKE) proto) ; done
#======================================================#	


#------------------------------ Maintain rules --------------------------------#
clean:
	-@rm -f $(CLEANEDFILES);
	@for i in ${CLEANDIRS} ; do \
		($(MAKE) -C $$i clean) ; \
	done

distclean: clean
	@rm -f `find . -name "\.depend*" -or -name "\.depend\.bak"`
	@rm -f cscope.out tags
#------------------------------------------------------------------------------#
install:
	@for i in ${INSTALLDIRS} ; do \
		($(MAKE) -C $$i install) ; \
	done
#------------------------------------------------------------------------------#
depend:
	@for i in ${DEPENDDIRS} ; do \
		($(MAKE) -C $$i depend); \
	done
#------------------------------------------------------------------------------#
dist: clean
	export VERSION && cd .. &&   tar czvf mrrocpp.tgz mrrocpp   && \
	mv mrrocpp.tgz "mrrocpp${VERSION}.tgz"

#==============================================================================#	

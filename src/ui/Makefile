ifeq ($(BUILD_TARGET), linux)
DIRS = src gtk
ALLDIRS = src gtk
INSTALLDIRS = gtk
else
DIRS = src
ALLDIRS = $(DIRS)
INSTALLDIRS = $(DIRS)
endif

shr:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) shr) ; done

app:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) app) ; done

proto:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) proto) ; done

clean:
	@for i in ${ALLDIRS} ; do (cd $$i && $(MAKE) clean) ; done

install:
	@for i in ${INSTALLDIRS} ; do (cd $$i && $(MAKE) install) ; done

depend:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) depend) ; done


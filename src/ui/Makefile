DIRS = src
ALLDIRS = src

UIDIR = src/gcc_ntox86

ifeq ($(TARGET), linux)
$(UIDIR):
	@$(MAKE) TARGET=linux -C $@
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
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) install) ; done

depend:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) depend) ; done


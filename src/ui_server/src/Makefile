DIRS = gcc_ntox86
ALLDIRS = gcc_ntox86

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

HOMEDIR = ../../..
SOURCES = ${wildcard *.cc}
include $(HOMEDIR)/depend.mk

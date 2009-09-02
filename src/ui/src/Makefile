include abplatform

ifeq ($(BUILD_TARGET), linux)
SOURCES = \
	ui.cc \
	ui_ecp_r_conveyor.cc \
	ui_ecp_r_irp6_common.cc \
	ui_ecp_r_speaker.cc
else

SOURCES = ${wildcard *.cc}

app:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) app) ; done

install:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) install) ; done
endif

shr:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) shr) ; done

proto:
	@for i in ${DIRS} ; do (cd $$i && $(MAKE) proto) ; done

clean:
	@for i in ${ALLDIRS} ; do (cd $$i && $(MAKE) clean) ; done

HOMEDIR = ../../..
include $(HOMEDIR)/depend.mk

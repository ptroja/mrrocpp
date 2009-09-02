include abplatform

ifeq ($(BUILD_TARGET), linux)
SOURCES = \
	ui.cc \
	ui_ecp_r_conveyor.cc \
	ui_ecp_r_irp6_common.cc \
	ui_ecp_r_speaker.cc
else
SOURCES = ${wildcard *.cc}
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

HOMEDIR = ../../..
include $(HOMEDIR)/depend.mk

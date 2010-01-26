ui_defs = mp.xml

ui_modules = mp.so

INSTALLDIRS = glade signals
SOURCEDIRS = generator $(INSTALLDIRS)

.PHONY: all clean install $(SOURCEDIRS)

all: $(SOURCEDIRS) ui.gtk ui.xml $(ui_defs) $(ui_modules)

# for QNX Photon Application Builder compatibility
shr: all

glade: generator
signals: generator

SOURCES = ${wildcard *.cc}

$(SOURCEDIRS):
	@$(MAKE) -C $@

install:

HOMEDIR = $(shell pwd)/../../..
include $(HOMEDIR)/common.mk

PACKAGES=libxml-2.0 gtk+-2.0 gtkmm-2.4 gthread-2.0
CPPFLAGS+=`pkg-config --cflags ${PACKAGES}`
CXXFLAGS+=-I.
LDFLAGS+=`pkg-config --libs ${PACKAGES}` -g -L${LIBDIR} \
	-lmessip -lecp \
	-lkinematicsconveyor -lkinematicsirp6m \
	-lkinematicsirp6ot -lkinematicsirp6p \
	-lkinematics -lmrrocpp
LDFLAGS+=-Wl,-export-dynamic
XSLTPROC=xsltproc robots.xml --xinclude
ECP_DIR = $(HOMEDIR)/src/ecp

ui.gtk: main.o xmlconfigurator.o \
	ui_model.o ui_config_entry.o ui_widget_entry.o \
	sr_console.o comm_thread.o \
	../src/gcc_ntox86/ui.o \
	../src/gcc_ntox86/threads.o
	$(CXX) -Wl,-rpath $(LIBDIR) -g -o $@ $^ ${LDFLAGS}

%.xml: %.glade
	gtk-builder-convert $< $@ 
	
%.so: %_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<
	
clean:
	@for i in ${INSTALLDIRS} ; do \
		($(MAKE) -C $$i clean) ; \
	done
	@rm -f *.xml ui.gtk *.o *.so

# TODO: ponizsza regule "install" trzeba ladniej przepisac
install: all
	@for i in ${INSTALLDIRS} ; do \
		($(MAKE) -C $$i install) ; \
	done
	$(INSTALL) ui.gtk start.sh *.xml *.so $(BINDIR)

include $(HOMEDIR)/depend.mk

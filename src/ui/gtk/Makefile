ui_defs = mp.xml

ui_modules = mp.so

INSTALLDIRS = glade signals
SOURCEDIRS = generator $(INSTALLDIRS)

.PHONY: all clean install depend $(SOURCEDIRS)

all: $(SOURCEDIRS) ui.gtk ui.xml $(ui_defs) $(ui_modules)

# for QNX Photon Application Builder compatibility
shr: all

glade: generator
signals: generator

$(SOURCEDIRS):
	@$(MAKE) -C $@

install:

HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

PACKAGES=libxml-2.0 gtk+-2.0 gtkmm-2.4 gthread-2.0
CPPFLAGS=-I${HOMEDIR}/src `pkg-config --cflags ${PACKAGES}` -DUSE_MESSIP_SRR
CXXFLAGS=-Wall -g -O0 -fpic -I $(HEADERSDIR) -I.
LDFLAGS+=`pkg-config --libs ${PACKAGES}` -g -L${HOMEDIR}/lib -lmessip -lecp -lmrrocpp -I $(HEADERSDIR) -I.
#LDFLAGS+=-export-dynamic
XSLTPROC=xsltproc robots.xml --xinclude
ECP_DIR = $(HOMEDIR)/src/ecp
HEADERSDIR = $(HOMEDIR)/src

ui.gtk: main.o xmlconfigurator.o \
	ui_model.o ui_config_entry.o ui_widget_entry.o \
	sr_console.o comm_thread.o \
	../src/gcc_ntox86/ui.o
	$(CXX) -Wl,-rpath `pwd`/../../../lib -g -o $@ $^ ${LDFLAGS}

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

# Directory with dependences.
DEPDIR = .deps
df = $(DEPDIR)/$(*F)
# Makedepend command.
MAKEDEPEND = mkdir -p .deps; gcc -M ${CPPFLAGS} -o $(DEPDIR)/$*.d $<

# Overwritten command for *.cc compilation - also creates dependency files (*.P) in adequate directory.
%.o : %.cc
	@$(MAKEDEPEND); \
	cp $(df).d $(df).P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $(df).d >> $(df).P; \
	rm -f $(df).d
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

# Wildcards used during dependency include.
SOURCES = ${wildcard *.cc}
DEPENDS = ${patsubst %.cc,$(DEPDIR)/%.P,${SOURCES}}

# Include created dependency files.
-include $(DEPENDS)

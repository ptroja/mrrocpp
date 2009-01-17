ui_defs = mp.xml

ui_modules = mp.so

SOURCEDIRS = generator glade signals

.PHONY: all clean install depend $(SOURCEDIRS)

all: $(SOURCEDIRS) ui.gtk ui.xml $(ui_defs) $(ui_modules)

$(SOURCEDIRS):
	@$(MAKE) -C $@

HOMEDIR = ../../..
include $(HOMEDIR)/common.mk

PACKAGES=libxml-2.0 gtk+-2.0 gtkmm-2.4 gthread-2.0
CPPFLAGS=-I${HOMEDIR}/include `pkg-config --cflags ${PACKAGES}` -DUSE_MESSIP_SRR
CXXFLAGS=-Wall -g -O0 -fpic
LDFLAGS+=`pkg-config --libs ${PACKAGES}` -export-dynamic -L${HOMEDIR}/lib -lmessip
XSLTPROC=xsltproc robots.xml --xinclude

ui.gtk: main.o configurator.o ui_model.o ui_config_entry.o sr_console.o
	$(CXX) -g -o $@ $^ ${LDFLAGS}

%.xml: %.glade
	gtk-builder-convert $< $@

%.so: %_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

clean:
	rm -f *.xml ui.gtk *.o *.so

# TODO: ponizsza regule "install" trzeba ladniej przepisac
install: all
	cp ui.gtk start.sh *.xml *.so $(HOMEDIR)/bin/

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

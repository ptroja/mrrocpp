all: ui ui.xml mp.xml festival.xml mech_xyz_axis.xml mp.so festival.so mech_xyz_axis.so 

PACKAGES=gtk+-2.0 libxml-2.0 gtkmm-2.4
CFLAGS=-Wall -g -O0 `pkg-config --cflags ${PACKAGES}`
CXXFLAGS=${CFLAGS} -fpic -shared
LDFLAGS=`pkg-config --libs ${PACKAGES}` -export-dynamic

ui: main.o configurator.o ui_model.o ui_config_entry.o
	$(CXX) -g -o $@ $^ ${LDFLAGS}

mp.so: mp_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

festival.so: festival_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

mech_xyz_axis.so: mech_xyz_axis_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

%.xml: %.glade
	gtk-builder-convert $< $@

clean:
	rm -f *.xml ui *.o *.so

# Directory with dependences.
DEPDIR = .deps
df = $(DEPDIR)/$(*F)
# Makedepend command.
MAKEDEPEND = mkdir -p .deps; gcc -M `pkg-config --cflags ${PACKAGES}` -o $(DEPDIR)/$*.d $<

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

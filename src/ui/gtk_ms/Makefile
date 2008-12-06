all: ui ui.xml mp.xml festival.xml mech_xyz_axis.xml mech_inc.xml mech_int.xml mech_xyz_euler.xml mech_ts_axis.xml mech_servo_algorithm.xml mp.so festival.so mech_xyz_axis.so mech_inc.so mech_int.so mech_xyz_euler.so mech_ts_axis.so mech_servo_algorithm.so

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

mech_inc.so: mech_inc_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

mech_int.so: mech_int_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

mech_xyz_euler.so: mech_xyz_euler_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

mech_ts_axis.so: mech_ts_axis_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

mech_servo_algorithm.so: mech_servo_algorithm_uimodule.o
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

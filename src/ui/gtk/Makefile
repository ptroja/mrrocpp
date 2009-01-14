ui_defs = mp.xml festival.xml \
	irp6m_axis_xyz.xml irp6m_inc.xml irp6m_int.xml irp6m_euler_xyz.xml irp6m_axis_ts.xml irp6m_servo_algorithm.xml \
	irp6o_axis_xyz.xml irp6o_inc.xml irp6o_int.xml irp6o_euler_xyz.xml irp6o_axis_ts.xml irp6o_servo_algorithm.xml \
	irp6p_axis_xyz.xml irp6p_inc.xml irp6p_int.xml irp6p_euler_xyz.xml irp6p_euler_ts.xml irp6p_servo_algorithm.xml \
	edp_irp6m_rcsc.xml

ui_modules = mp.so festival.so \
	irp6m_axis_xyz.so irp6m_inc.so irp6m_int.so irp6m_euler_xyz.so irp6m_axis_ts.so irp6m_servo_algorithm.so \
	irp6o_axis_xyz.so irp6o_inc.so irp6o_int.so irp6o_euler_xyz.so irp6o_axis_ts.so irp6o_servo_algorithm.so \
	irp6p_axis_xyz.so irp6p_inc.so irp6p_int.so irp6p_euler_xyz.so irp6p_euler_ts.so irp6p_servo_algorithm.so \
	edp_irp6m_rcsc.so

all: ui.gtk ui.xml $(ui_defs) $(ui_modules)

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
	cp ui.gtk start.sh *.xml *.so ../../../bin/

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

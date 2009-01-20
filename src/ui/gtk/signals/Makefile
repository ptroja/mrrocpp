ui_modules = festival.so \
	irp6m_axis_xyz.so irp6m_inc.so irp6m_int.so irp6m_euler_xyz.so irp6m_axis_ts.so irp6m_servo_algorithm.so \
	irp6o_axis_xyz.so irp6o_inc.so irp6o_int.so irp6o_euler_xyz.so irp6o_axis_ts.so irp6o_servo_algorithm.so \
	irp6p_axis_xyz.so irp6p_inc.so irp6p_int.so irp6p_euler_xyz.so irp6p_euler_ts.so irp6p_servo_algorithm.so \
	conveyor_inc.so conveyor_int.so conveyor_servo_algorithm.so \
	edp_irp6m.so edp_irp6o.so edp_irp6p.so edp_conveyor.so 

HOMEDIR = ../../../..
include $(HOMEDIR)/common.mk

PACKAGES=libxml-2.0 gtk+-2.0 gtkmm-2.4 gthread-2.0
CPPFLAGS=-I${HOMEDIR}/include -I.. `pkg-config --cflags ${PACKAGES}` -DUSE_MESSIP_SRR
CXXFLAGS=-Wall -g -O0 -fpic
LDFLAGS+=`pkg-config --libs ${PACKAGES}` -export-dynamic -L${HOMEDIR}/lib -lmessip
ECP_DIR = $(HOMEDIR)/src/ecp

MRROCOBJ = $(ECP_DIR)/irp6_mechatronika/ecp_local.o $(ECP_DIR)/common/libecp.a $(LIBDIR)/libmrrocpp.a

all: $(ui_modules)

%.so: %_widget.o $(MRROCOBJ)
	$(CXX) ${LDFLAGS} -shared -o $@ $(MRROCOBJ) $<
	
%.so: %_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<

clean:
	rm -f *.o *.so

# TODO: ponizsza regule "install" trzeba ladniej przepisac
install: all
	cp *.so $(HOMEDIR)/bin/

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

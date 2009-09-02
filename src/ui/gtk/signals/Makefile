ui_modules = festival.so \
	edp_irp6m.so edp_irp6o.so edp_irp6p.so edp_conveyor.so  

HOMEDIR = ../../../..
include $(HOMEDIR)/common.mk

PACKAGES=libxml-2.0 gtk+-2.0 gtkmm-2.4 gthread-2.0
CPPFLAGS=-I${HOMEDIR}/src -I.. `pkg-config --cflags ${PACKAGES}` -DUSE_MESSIP_SRR
CXXFLAGS=-Wall -g -O0 -fpic -I $(HEADERSDIR) -I.
LDFLAGS+=`pkg-config --libs ${PACKAGES}` -export-dynamic -L${HOMEDIR}/lib
ECP_DIR = $(HOMEDIR)/src/ecp
HEADERSDIR = $(HOMEDIR)/src

#LIBS = `pwd`/../../../../lib/libecp.so `pwd`/../../../../lib/libmrrocpp.so `pwd`/../../../../lib/libmessip.so

all: $(ui_modules)

IRP6_COMMON_OBJS = \
	$(ECP_DIR)/irp6_on_track/ecp_r_irp6ot.o \
	$(ECP_DIR)/irp6_postument/ecp_r_irp6p.o \
	$(ECP_DIR)/irp6_mechatronika/ecp_r_irp6m.o \
	$(ECP_DIR)/polycrank/ecp_r_polycrank.o \

edp_irp6m.so: edp_irp6m_uimodule.o \
	$(IRP6_COMMON_OBJS) $(UI_DIR)/ui_ecp_r_irp6_common.o
	$(CXX) ${LDFLAGS} -shared -o $@ $(LIBS) $^

edp_irp6o.so: edp_irp6o_uimodule.o \
	$(IRP6_COMMON_OBJS) $(UI_DIR)/ui_ecp_r_irp6_common.o
	$(CXX) ${LDFLAGS} -shared -o $@ $(LIBS) $^

edp_irp6p.so: edp_irp6p_uimodule.o \
	$(IRP6_COMMON_OBJS) $(UI_DIR)/ui_ecp_r_irp6_common.o
	$(CXX) ${LDFLAGS} -shared -o $@ $(LIBS) $^

edp_conveyor.so: edp_conveyor_uimodule.o \
	$(ECP_DIR)/conveyor/ecp_r_conv.o \
	$(UI_DIR)/ui_ecp_r_conveyor.o
	$(CXX) ${LDFLAGS} -shared -o $@ $(LIBS) $^
	
festival.so: festival_uimodule.o
	$(CXX) ${LDFLAGS} -shared -o $@ $<
	
clean:
	rm -f *.o *.so

# TODO: ponizsza regule "install" trzeba ladniej przepisac
install: all
	$(INSTALL) *.so $(BINDIR)

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

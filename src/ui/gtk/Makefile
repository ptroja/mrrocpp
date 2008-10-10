all: ui ui.xml

CFLAGS=-g -O0 `pkg-config --cflags gtk+-2.0 libxml-2.0`
CXXFLAGS=${CFLAGS}
LDFLAGS=`pkg-config --libs gtk+-2.0 libxml-2.0` -export-dynamic

ui: main.o configurator.o ui_model.o
	$(CXX) -g -o $@ $^ ${LDFLAGS}

ui.xml: ui.glade
	gtk-builder-convert $< $@

clean:
	rm -f ui.xml ui *.o

# Directory with dependences.
DEPDIR = .deps
df = $(DEPDIR)/$(*F)
# Makedepend command.
MAKEDEPEND = mkdir -p .deps; gcc -M `pkg-config --cflags gtk+-2.0 libxml-2.0` -o $(DEPDIR)/$*.d $<

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


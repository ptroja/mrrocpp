ui_defs = festival.xml \
	edp_irp6m.xml edp_irp6o.xml edp_irp6p.xml edp_conveyor.xml

HOMEDIR = ../../../..
include $(HOMEDIR)/common.mk

all: $(ui_defs)

%.xml: %.glade
	gtk-builder-convert $< $@

install: all
	$(INSTALL) *.xml $(BINDIR)

clean:
	@rm -f *.xml

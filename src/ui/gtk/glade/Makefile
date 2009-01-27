ui_defs = festival.xml \
	edp_irp6m.xml edp_irp6o.xml edp_irp6p.xml edp_conveyor.xml 

HOMEDIR = ../../../..

all: $(ui_defs)

%.xml: %.glade
	gtk-builder-convert $< $@

install: all
	cp *.xml $(HOMEDIR)/bin/

clean:
	rm -f *.xml

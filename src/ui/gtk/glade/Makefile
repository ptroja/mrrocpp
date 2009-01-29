ui_defs = festival.xml \
	irp6m_axis_xyz.xml irp6m_inc.xml irp6m_int.xml irp6m_euler_xyz.xml irp6m_axis_ts.xml irp6m_euler_ts.xml irp6m_servo_algorithm.xml \
	irp6o_axis_xyz.xml irp6o_inc.xml irp6o_int.xml irp6o_euler_xyz.xml irp6o_axis_ts.xml irp6o_euler_ts.xml irp6o_servo_algorithm.xml \
	irp6p_axis_xyz.xml irp6p_inc.xml irp6p_int.xml irp6p_euler_xyz.xml irp6p_axis_ts.xml irp6p_euler_ts.xml irp6p_servo_algorithm.xml \
	conveyor_inc.xml conveyor_int.xml conveyor_servo_algorithm.xml \
	edp_irp6m.xml edp_irp6o.xml edp_irp6p.xml edp_conveyor.xml

HOMEDIR = ../../../..

all: $(ui_defs)

%.xml: %.glade
	gtk-builder-convert $< $@

install: all
	cp *.xml $(HOMEDIR)/bin/

clean:
	rm -f *.xml


#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "edp_irp6o_uimodule.h"

#include "ui/ui_ecp_r_irp6_common.h"

edp_irp6o::edp_irp6o(ui_config_entry &entry)
{	
}

edp_irp6o::~edp_irp6o()
{
}

static edp_irp6o *edp_ontrack;


extern "C" 
{ 
	void  on_combobox1_changed_ontrack(GtkComboBox *comboBox, gpointer userdata)  
	{
		ui_config_entry & comboEntry = *(ui_config_entry *) userdata;
		GtkBuilder & builder = (comboEntry.getBuilder());
		
		GtkScrolledWindow * scrolled = GTK_SCROLLED_WINDOW (gtk_builder_get_object(&builder, "scrolledwindow1"));

		//if the child exists, destroy it
		if (gtk_bin_get_child(GTK_BIN(scrolled))!=NULL)
		{
			GtkWidget* child = gtk_bin_get_child(GTK_BIN(scrolled));
			gtk_widget_destroy(child);
		}
		
		ui_widget_entry * ChoseEntry;
		gboolean isFile = 0;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox);

		switch (choice)
		{
		case 0: std::cout << "Servo algorithm window chosen" << std::endl; ChoseEntry = comboEntry.getWidget(0); isFile = 1; break;
		case 1: std::cout << "Internal window chosen" << std::endl; ChoseEntry = comboEntry.getWidget(1); isFile = 1; break;
		case 2: std::cout << "Increment window chosen" << std::endl; ChoseEntry = comboEntry.getWidget(2); isFile = 1; break;
		case 3: std::cout << "XYZ Angle Axis window chosen" << std::endl; ChoseEntry = comboEntry.getWidget(3); isFile = 1; break;
		case 4: std::cout << "XYZ Euler ZYZ window chosen" << std::endl; ChoseEntry = comboEntry.getWidget(4); isFile = 1; break;
		case 5: std::cout << "TS Angle Axis window chosen" << std::endl; ChoseEntry = comboEntry.getWidget(5); isFile = 1; break;
		case 6:  break;
		default: std::cout << "Something is not working properly!" << std::endl;
		}
		
		if (isFile)
		{
			GtkBuilder & chosenFileBuilder = ((*ChoseEntry).getBuilder());
	
			GtkWidget* chosenWindow = GTK_WIDGET (gtk_builder_get_object (&chosenFileBuilder, "window"));
			g_assert(chosenWindow);
			
			GtkWidget* windowWithoutParent = gtk_bin_get_child(GTK_BIN(chosenWindow));
			gtk_widget_unparent(windowWithoutParent);
			
			gtk_scrolled_window_add_with_viewport (scrolled, windowWithoutParent);
		}
		
	}	

	void ui_module_init(ui_config_entry &entry) 
	{
		edp_ontrack = new edp_irp6o(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
		
		ui_widget_entry * widgetEntry1 = new ui_widget_entry("irp6o_servo_algorithm.xml"); entry.addWidget(widgetEntry1);
		ui_widget_entry * widgetEntry2 = new ui_widget_entry("irp6o_int.xml"); entry.addWidget(widgetEntry2);
		ui_widget_entry * widgetEntry3 = new ui_widget_entry("irp6o_inc.xml"); entry.addWidget(widgetEntry3);
		ui_widget_entry * widgetEntry4 = new ui_widget_entry("irp6o_axis_xyz.xml"); entry.addWidget(widgetEntry4);
		ui_widget_entry * widgetEntry5 = new ui_widget_entry("irp6o_euler_xyz.xml"); entry.addWidget(widgetEntry5);
		ui_widget_entry * widgetEntry6 = new ui_widget_entry("irp6o_axis_ts.xml"); entry.addWidget(widgetEntry6);
		
		
		robot = new ui_common_robot(
				ui_model::instance().getConfigurator(),
				&ui_model::instance().getEcpSr()
				,ROBOT_IRP6_ON_TRACK
				);
		
        robot->get_controller_state(&state);
	}

	void ui_module_unload(void) 
	{
		if (edp_ontrack) 
		{
			delete edp_ontrack;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

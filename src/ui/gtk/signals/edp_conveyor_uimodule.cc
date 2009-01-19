
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "edp_conveyor_uimodule.h"


edp_conveyor::edp_conveyor(ui_config_entry &entry)
{	
}

edp_conveyor::~edp_conveyor()
{
}

static edp_conveyor *edp_conveyorRobot;


extern "C" 
{ 
	void  on_combobox1_changed_conveyorRobot(GtkComboBox *comboBox, gpointer userdata)  
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
		case 3:  break;
		case 4:  break;
		case 5:  break;
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
		edp_conveyorRobot = new edp_conveyor(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
		
		ui_widget_entry * widgetEntry1 = new ui_widget_entry("conveyor_servo_algorithm.xml");
		entry.addWidget(widgetEntry1);
		ui_widget_entry * widgetEntry2 = new ui_widget_entry("conveyor_int.xml");
		entry.addWidget(widgetEntry2);
		ui_widget_entry * widgetEntry3 = new ui_widget_entry("conveyor_inc.xml");
		entry.addWidget(widgetEntry3);
		
		
	}

	void ui_module_unload(void) 
	{
		if (edp_conveyorRobot) 
		{
			delete edp_conveyorRobot;
		}
		fprintf(stderr, "config %s unloaded\n", __FILE__);
	}
}

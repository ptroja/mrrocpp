#include <gtk/gtk.h>
#include <glib.h>


#include <iostream>

#include "ui_model.h"
#include "edp_irp6m_rcsc.h"

edp_ip6m_rcsc::edp_ip6m_rcsc(ui_config_entry &entry)
{
	
}

edp_ip6m_rcsc::~edp_ip6m_rcsc()
{
	
}

static edp_ip6m_rcsc *edp_mechatronika;

extern "C" 
{ 
	void  on_combobox1_changed(GtkComboBox *comboBox, gpointer userdata)  
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
		
		const gchar * ChosenFile;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox);
		 
		switch (choice)
		{
		case 0: std::cout << "Servo algorithm window chosen" << std::endl; ChosenFile = "irp6m_servo_algorithm.xml"; break;
		case 1: std::cout << "Internal window chosen" << std::endl; ChosenFile = "irp6m_int.xml"; break;
		case 2: std::cout << "Increment window chosen" << std::endl; ChosenFile = "irp6m_inc.xml"; break;
		case 3: std::cout << "XYZ Euler ZYZ window chosen" << std::endl; ChosenFile = "irp6m_euler_xyz.xml"; break;
		case 4: std::cout << "XYZ Angle Axis window chosen" << std::endl; ChosenFile = "irp6m_axis_xyz.xml"; break;
		case 5: std::cout << "TS Angle Axis window chosen" << std::endl; ChosenFile = "irp6m_axis_ts.xml"; break;
		default: std::cout << "Something is not working properly!" << std::endl;
		}
		
		GtkBuilder* chosenFileBuilder = gtk_builder_new();
		GError *err = NULL;
		if (gtk_builder_add_from_file(chosenFileBuilder, ChosenFile, &err) == 0) 
		{
			fprintf (stderr, "Unable to read file %s: %s\n", ChosenFile, err->message);
			g_error_free (err);

			// TODO: throw(...)
		}
		g_assert(chosenFileBuilder);
		
		GModule* module = g_module_open("./irp6m_servo_algorithm", (GModuleFlags) G_MODULE_BIND_LAZY);

		if(module) 
		{
			gtk_builder_connect_signals(chosenFileBuilder, userdata);
			gpointer symbol;
			if (g_module_symbol(module, "ui_module_init", &symbol)) 
			{
				typedef void (*ui_module_init_t)(ui_config_entry &entry);

				//ui_module_init_t ui_module_init = (ui_module_init_t) symbol;

				//ui_config_entry & newEntry = *(ui_config_entry *) userdata;

			}
			else 
			{
			g_warning("failed to open module %s.%s\n", "./irp6m_axis_ts", G_MODULE_SUFFIX );
			}			
		}

		GtkWidget* chosenWindow = GTK_WIDGET (gtk_builder_get_object (chosenFileBuilder, "window"));
		g_assert(chosenWindow);
		
		GtkWidget* windowWithoutParent = gtk_bin_get_child(GTK_BIN(chosenWindow));
		gtk_widget_unparent(windowWithoutParent);
		
		gtk_scrolled_window_add_with_viewport (scrolled, windowWithoutParent);
		
	}	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		edp_mechatronika = new edp_ip6m_rcsc(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (edp_mechatronika) 
		{
			delete edp_mechatronika;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}





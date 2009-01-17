
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "conveyor_inc_uimodule.h"


edp_conveyor_inc::edp_conveyor_inc(ui_config_entry &entry) 
{
}

static edp_conveyor_inc *inc_conveyor;


extern "C"
{
	void on_arrow_button_clicked_conveyor_inc (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla conveyor inc" << std::endl;
	}
	
	void on_read_button_clicked_conveyor_inc (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla conveyor inc" << std::endl;
	}
	
	void on_execute_button_clicked_conveyor_inc (GtkButton* button, gpointer user_data)
	{
		std::cout << "Execute move dla conveyor inc" << std::endl;
	}
	
	void on_export_button_clicked_conveyor_inc (GtkButton* button, gpointer user_data)
	{
		std::cout << "Export dla conveyor inc" << std::endl;
	}
	
	void on_import_button_clicked_conveyor_inc (GtkButton* button, gpointer user_data)
	{
		std::cout << "Import dla conveyor inc" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		inc_conveyor = new edp_conveyor_inc(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (inc_conveyor) 
		{
			delete inc_conveyor;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
	

	void on_button1_clicked_conveyor_inc (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button1 dla conveyor inc" << std::endl;
	}
    

	void on_button2_clicked_conveyor_inc (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button2 dla conveyor inc" << std::endl;
	}
    

}


#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "conveyor_int_uimodule.h"


edp_conveyor_int::edp_conveyor_int(ui_config_entry &entry) 
{
}

static edp_conveyor_int *int_conveyor;


extern "C"
{
	void on_arrow_button_clicked_conveyor_int (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla conveyor int" << std::endl;
	}
	
	void on_read_button_clicked_conveyor_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla conveyor int" << std::endl;
	}
	
	void on_execute_button_clicked_conveyor_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Execute move dla conveyor int" << std::endl;
	}
	
	void on_export_button_clicked_conveyor_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Export dla conveyor int" << std::endl;
	}
	
	void on_import_button_clicked_conveyor_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Import dla conveyor int" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		int_conveyor = new edp_conveyor_int(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (int_conveyor) 
		{
			delete int_conveyor;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
	

	void on_button1_clicked_conveyor_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button1 dla conveyor int" << std::endl;
	}
    

	void on_button2_clicked_conveyor_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button2 dla conveyor int" << std::endl;
	}
    

}


#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "conveyor_servo_algorithm_uimodule.h"


edp_conveyor_servo_algorithm::edp_conveyor_servo_algorithm(ui_config_entry &entry) 
{
}

static edp_conveyor_servo_algorithm *servo_conveyor;


extern "C"
{
	void on_arrow_button_clicked_conveyor_servo (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla conveyor servo" << std::endl;
	}
	
	void on_read_button_clicked_conveyor_servo (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla conveyor servo" << std::endl;
	}
	
	void on_set_button_clicked_conveyor_servo (GtkButton* button, gpointer user_data)
	{
		std::cout << "ustaw wartosci dla conveyor servo" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		servo_conveyor = new edp_conveyor_servo_algorithm(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (servo_conveyor) 
		{
			delete servo_conveyor;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

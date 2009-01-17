
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6o_servo_algorithm_uimodule.h"


edp_irp6o_servo_algorithm::edp_irp6o_servo_algorithm(ui_config_entry &entry) 
{
}

static edp_irp6o_servo_algorithm *servo_ontrack;


extern "C"
{
	void on_arrow_button_clicked_ontrack_servo (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla ontrack servo" << std::endl;
	}
	
	void on_read_button_clicked_ontrack_servo (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla ontrack servo" << std::endl;
	}
	
	void on_set_button_clicked_ontrack_servo (GtkButton* button, gpointer user_data)
	{
		std::cout << "ustaw wartosci dla ontrack servo" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		servo_ontrack = new edp_irp6o_servo_algorithm(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (servo_ontrack) 
		{
			delete servo_ontrack;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}


#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6p_euler_ts_uimodule.h"


edp_irp6p_euler_ts::edp_irp6p_euler_ts(ui_config_entry &entry) 
{
}

static edp_irp6p_euler_ts *euler_ts_postument;


extern "C"
{
	void on_arrow_button_clicked_postument_euler_ts (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla postument euler_ts" << std::endl;
	}
	
	void on_read_button_clicked_postument_euler_ts (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla postument euler_ts" << std::endl;
	}
	
	void on_set_button_clicked_postument_euler_ts (GtkButton* button, gpointer user_data)
	{
		std::cout << "ustaw wartosci dla postument euler_ts" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		euler_ts_postument = new edp_irp6p_euler_ts(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (euler_ts_postument) 
		{
			delete euler_ts_postument;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

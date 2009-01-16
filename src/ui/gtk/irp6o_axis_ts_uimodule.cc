
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6o_axis_ts_uimodule.h"


edp_irp6o_axis_ts::edp_irp6o_axis_ts(ui_config_entry &entry) 
{
}

static edp_irp6o_axis_ts *axis_ts_ontrack;


extern "C"
{
	void on_arrow_button_clicked_ontrack_axis_ts (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla ontrack axis_ts" << std::endl;
	}
	
	void on_read_button_clicked_ontrack_axis_ts (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla ontrack axis_ts" << std::endl;
	}
	
	void on_set_button_clicked_ontrack_axis_ts (GtkButton* button, gpointer user_data)
	{
		std::cout << "ustaw wartosci dla ontrack axis_ts" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		axis_ts_ontrack = new edp_irp6o_axis_ts(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (axis_ts_ontrack) 
		{
			delete axis_ts_ontrack;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}


#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6m_axis_ts_uimodule.h"


edp_irp6m_axis_ts::edp_irp6m_axis_ts(ui_config_entry &entry) 
{
}

static edp_irp6m_axis_ts *axis_ts_mechatronika;


extern "C"
{
	void on_arrow_button_clicked_mechatronika_axis_ts (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla mechatronika axis_ts" << std::endl;
	}
	
	void on_read_button_clicked_mechatronika_axis_ts (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla mechatronika axis_ts" << std::endl;
	}
	
	void on_set_button_clicked_mechatronika_axis_ts (GtkButton* button, gpointer user_data)
	{
		std::cout << "ustaw wartosci dla mechatronika axis_ts" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		axis_ts_mechatronika = new edp_irp6m_axis_ts(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (axis_ts_mechatronika) 
		{
			delete axis_ts_mechatronika;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

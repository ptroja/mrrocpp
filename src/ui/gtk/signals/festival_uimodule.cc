
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "festival_uimodule.h"


festival::festival(ui_config_entry &entry) 
{
}

static festival *festivalPanel;


extern "C"
{
	void on_say_button_clicked_festival (GtkButton* button, gpointer userdata)
	{
		std::cout << "Wcisniety przycisk Say dla festival" << std::endl;
	}
	
	void ui_module_init(ui_config_entry &entry) 
	{
		festivalPanel = new festival(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (festivalPanel) 
		{
			delete festivalPanel;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

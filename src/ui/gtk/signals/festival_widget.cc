#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>
#include "ui_model.h"
#include "festival_widget.h"



festival::festival(ui_widget_entry &entry) 
{
}

static festival *festivalPanel;


extern "C"
{
	void on_say_button_clicked_festival (GtkButton* button, gpointer userdata)
	{
		std::cout << "Say button clicked in festival window" <<std::endl;
	}	
	
	void ui_widget_init(ui_widget_entry &entry) 
	{
		festivalPanel = new festival(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (festivalPanel) 
		{
			delete festivalPanel;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}
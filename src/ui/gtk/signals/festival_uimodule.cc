#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>
#include "ui_model.h"
#include "festival_uimodule.h"



festival::festival(ui_config_entry &entry) 
{
	GtkBuilder & builder = (entry.getBuilder());

	GtkComboBox *LangCombo = GTK_COMBO_BOX(gtk_builder_get_object(&builder, "LanguageCombo"));
	gtk_combo_box_set_active(LangCombo, 0);
}

static festival *festivalPanel;


extern "C"
{
	void on_say_button_clicked_festival (GtkButton* button, gpointer userdata)
	{
		std::cout << "Say button clicked in festival window" <<std::endl;
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

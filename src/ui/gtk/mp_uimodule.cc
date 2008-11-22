#include <gtk/gtk.h>
#include <glib.h>

#include <iostream>

#include "ui_config_entry.h"
#include "ui_model.h"

void mp_module_init(ui_config_entry &entry) {

	GtkBuilder & builder = entry.getBuilder();

	ui_config_entry &root = ui_model::instance().getRootNode();

	std::vector <ui_config_entry *> ecps = ui_model::instance().getRootNode().getChildByType(ui_config_entry::ECP);
}

extern "C" {

	void on_button1_activate(GtkObject *object, gpointer user_data)
	{
		g_warn_if_reached();
	}


	void on_button1_clicked(GtkObject *object, gpointer user_data)
	{
		g_warn_if_reached();
	}

	void ui_module_init(ui_config_entry &entry) {
		mp_module_init(entry);
	}

}

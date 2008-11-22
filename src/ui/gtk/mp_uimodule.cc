#include <gtk/gtk.h>
#include <glib.h>

#include <stdio.h>

#include "ui_config_entry.h"

extern "C" {

	void on_button1_activate(GtkObject *object, gpointer user_data)
	{
		g_warn_if_reached();
	}


	void on_button1_clicked(GtkObject *object, gpointer user_data)
	{
		g_warn_if_reached();
	}

//	void ui_module_init(ui_config_entry &entry) {
//		//g_warn_if_reached();
//	}

	void ui_module_init(void *entry) {
		printf("qqq\n");
	}

}

void mp_module_init(void *entry);

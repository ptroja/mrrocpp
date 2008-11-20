#include <gtk/gtk.h>
#include <glib.h>

#include <stdio.h>

extern "C" {

	void on_button1_activate(GtkObject *object, gpointer user_data)
	{
		g_warn_if_reached();
	}


	void on_button1_clicked(GtkObject *object, gpointer user_data)
	{
		g_warn_if_reached();
	}

}

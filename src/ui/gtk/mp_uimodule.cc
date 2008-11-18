#include <gtk/gtk.h>
#include <glib.h>

#include <stdio.h>

extern "C" {

	void on_button1_activate(GtkObject *object, gpointer user_data)
	{
		printf("%s@%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
	}


	void on_button1_clicked(GtkObject *object, gpointer user_data)
	{
		printf("%s@%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
	}

}

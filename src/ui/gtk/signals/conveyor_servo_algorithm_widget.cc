
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "conveyor_servo_algorithm_widget.h"


edp_conveyor_servo_algorithm::edp_conveyor_servo_algorithm(ui_widget_entry &entry)
{
}

static edp_conveyor_servo_algorithm *servo_conveyorRobot;


extern "C"
{
	void on_arrow_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla conveyorRobot servo" << std::endl;

        ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());

        GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
        g_return_if_fail(entry1);
        const gchar * sth = gtk_entry_get_text(entry1);
        std::cout << sth << std::endl;
	}

	void on_read_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla conveyorRobot servo" << std::endl;
	}

	void on_set_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer user_data)
	{
		std::cout << "ustaw wartosci dla conveyorRobot servo" << std::endl;
	}


	void ui_widget_init(ui_widget_entry &entry)
	{
		servo_conveyorRobot = new edp_conveyor_servo_algorithm(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void)
	{
		if (servo_conveyorRobot)
		{
			delete servo_conveyorRobot;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
}

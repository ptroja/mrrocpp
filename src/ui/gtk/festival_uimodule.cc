#include <gtk/gtk.h>
#include <glib.h>

#include <iostream>

#include "ui_config_entry.h"
#include "ui_model.h"

class festival_ui {
	int i;
};

extern "C" {
	void send_cmd_to_festival(GtkButton *button, gpointer user_data);
}

void send_cmd_to_festival(GtkButton *button, gpointer user_data) {
	printf("send_cmd_to_festival()\n");
}

extern "C" {

	void ui_module_init(ui_config_entry &entry) {
		printf("inicjalizacja festivala - pewnie trzeba nawiazac polaczenie z ECP\n");
	}

}

#include <gtk/gtk.h>
#include <glib.h>
#include <gtkmm.h>

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

class FestivalPanel {
	public:
		FestivalPanel(ui_config_entry &entry);
};

FestivalPanel::FestivalPanel(ui_config_entry &entry) {
	GtkBuilder & builder = (entry.getBuilder());

	GtkComboBox *LangCombo = GTK_COMBO_BOX(gtk_builder_get_object(&builder, "LanguageCombo"));
	gtk_combo_box_set_active(LangCombo, 0);
}

static FestivalPanel *panel;

extern "C" {

	void ui_module_init(ui_config_entry &entry) {
		panel = new FestivalPanel(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) {
		if (panel) {
			delete panel;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

#include "ui_model.h"

ui_model& ui_model::instance()
{ static ui_model * pointerToTheSingletonInstance = new ui_model;
  return *pointerToTheSingletonInstance;
}

void ui_model::add_ui_config_entry(ui_config_entry::ui_config_entry_type entry_type, const char *program, const char *node) {
	ui_config_entry *entry = new ui_config_entry(entry_type, program, node);

	switch (entry_type) {
		GNode *child;
		case ui_config_entry::MP_PARENT:
		case ui_config_entry::SENSORS_PARENT:
		case ui_config_entry::EFFECTORS_PARENT:
			break;
		case ui_config_entry::SENSOR:
		case ui_config_entry::EFFECTOR:
			break;
		case ui_config_entry::MP:
			break;
		case ui_config_entry::VSP:
			break;
		case ui_config_entry::ECP:
			break;
		case ui_config_entry::EDP:
			break;
	}
}

ui_model::ui_model() {
	add_ui_config_entry(ui_config_entry::MP_PARENT, "Master Process");
	add_ui_config_entry(ui_config_entry::SENSORS_PARENT, "Sensors");
	add_ui_config_entry(ui_config_entry::EFFECTORS_PARENT, "Effectors");

/*
	const char *main_rows[] = {
		"Master Process",
		"Sensors",
		"Effectors"
	};

	for (int i = 0; i < N_ROWS; i++) {
		GtkTreeIter iter1;  Parent iter
		gtk_tree_store_append(store, &iter1, NULL);  Acquire a top-level iterator
		gtk_tree_store_set(store, &iter1, NAME_COLUMN, main_rows[i], IS_RUNNING_COLUMN, FALSE, -1);
	}
*/
}

ui_model::~ui_model() {
}

#ifndef __UI_CONFIG_H

#include <iostream>
#include <map>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>

class ui_config_entry
{
	public:
		const std::string id;
		const std::string program_name;
		const std::string node_name;
		std::string panel_model_file;
		std::string panel_widget_name;

		bool is_running;

		GtkBuilder *panel_builder;
		GtkWidget *panel_widget;

		enum ui_config_entry_type {
			MP_PARENT,
			MP,
			SENSORS_PARENT,
			SENSOR,
			VSP,
			EFFECTORS_PARENT,
			EFFECTOR,
			ECP,
			EDP
		} type;

		ui_config_entry(ui_config_entry_type entry_type, const char *program = NULL, const char *node = NULL) :
			type(entry_type), program_name(program), node_name(node ? node : "")
		{
		}

		//! map of children nodes
		std::map <std::string, ui_config_entry> childs;
};

class ui_model
{
	public:
		static ui_model& instance();

		void add_ui_config_entry(ui_config_entry::ui_config_entry_type entry_type, const char *program, const char *node = NULL);

	private:
		ui_model();

		//! maps of main menus
		std::map <std::string, ui_config_entry> menus;

		ui_model(ui_model const&); //not defined, not copyable
		ui_model& operator=(ui_model const&); //not defined, not assignable
		~ui_model();
};

namespace
{ struct ForceSingletonInitialization
  { ForceSingletonInitialization() { ui_model::instance(); }
  } instance;
}

#endif /* __UI_CONFIG_H */

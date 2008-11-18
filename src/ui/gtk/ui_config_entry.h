#ifndef __UI_CONFIG_ENTRY_H
#define __UI_CONFIG_ENTRY_H

#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>

class ui_config_entry
{
	public:
		// these should be const's but it make std::vector fail
		// see: http://groups.google.com/group/comp.lang.c++/msg/2cc5480095ca9b73?pli=1
		std::string id;
		std::string program_name;
		std::string node_name;
		std::string panel_model_file;
		std::string panel_widget_name;

		bool is_running;

		enum ui_config_entry_type {
			ROOT,
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

		ui_config_entry();
		ui_config_entry(ui_config_entry_type _type, const char *program = NULL, const char *node = NULL, const char *ui_def = NULL);
		~ui_config_entry();

		GtkTreeIter getTree_iter() const
		{
			return tree_iter;
		}

		void setTree_iter(GtkTreeIter tree_iter)
		{
			this->tree_iter = tree_iter;
		}

		void add_child(ui_config_entry & child);

		void remove_childs(void);

		bool is_empty(void);

		//! children nodes
		std::vector <ui_config_entry *> children;

		void show_page(GtkNotebook *notebook);

	private:

		GtkBuilder *builder;
		GtkWidget *window;
		GModule *module;

		bool page_visible;

		GtkTreeIter tree_iter;
};

#endif /* __UI_CONFIG_ENTRY_H */

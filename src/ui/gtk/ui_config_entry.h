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
		const std::string id;
		const std::string program_name;
		const std::string node_name;

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

		GtkTreeIter & getTree_iter() const;
		void setTree_iter(GtkTreeIter & _tree_iter);

		void add_child(ui_config_entry & child);

		void remove_childs(void);

		int childrens(void);

		//! children nodes
		std::vector <ui_config_entry *> children;

		void show_page(bool visible);

		GtkBuilder & getBuilder(void) {
			if (builder) {
				return *((GtkBuilder*)&this->builder);
			} else {
				throw ("no GtkBuilder in module " + program_name);
			}
		}

		std::vector <ui_config_entry *> getChildByType(ui_config_entry_type _type);

	private:

		GtkBuilder *builder;
		GtkWidget *window;
		GModule *module;

		bool page_visible;

		GtkTreeIter tree_iter;

		//! pagetab widgets
		GtkHBox *hbox;
		GtkImage *tabicon;
		GtkLabel *tablabel;
		GtkButton *tabbutton;
		GtkImage *tabcloseicon;
		GtkWidget *content;
};

#endif /* __UI_CONFIG_ENTRY_H */

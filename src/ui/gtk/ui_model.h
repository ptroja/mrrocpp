#ifndef __UI_CONFIG_H

#include <iostream>
#include <vector>

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
	ui_config_entry(ui_config_entry_type _type, const char *program = NULL, const char *node = NULL);

    GtkTreeIter getTree_iter() const
    {
        return tree_iter;
    }

    void setTree_iter(GtkTreeIter tree_iter)
    {
        this->tree_iter = tree_iter;
    }

    //void add_child(ui_config_entry & child);

	private:

		//! children nodes
		std::vector <ui_config_entry> childs;

		GtkTreeIter tree_iter;
};

class ui_model
{
	public:
		static ui_model& instance();

		void clear(void);

		ui_config_entry & add_ui_config_entry(ui_config_entry & parent, ui_config_entry::ui_config_entry_type entry_type, const char *program_name, const char *node_name_name = NULL);

		GtkTreeStore *getStore() const
		{
			return store;
		}

		ui_config_entry & getRootNode()
		{
			return root_entry;
		}

		enum TREE_VIEW_COLUMNS
		{
			NAME_COLUMN, /* name */
			NODE_NAME_COLUMN, /* Executing node of the process */
			IS_RUNNING_COLUMN, /* Is currently running? */
			N_COLUMNS
		};

		enum TREE_VIEW_MAIN_ROWS
		{
			MASTER_PROCESS_MAIN_ROW, SENSORS_MAIN_ROW, EFFECTORS_MAIN_ROW, N_ROWS
		};

	private:
		ui_model();

		ui_config_entry root_entry;

		// GtkTreeView model
		GtkTreeStore *store;

		ui_model(ui_model const&); //not defined, not copyable
		ui_model& operator=(ui_model const&); //not defined, not assignable
		~ui_model();
};

/*
namespace {
struct ForceSingletonInitialization
{
	ForceSingletonInitialization()
	{
		printf("qpa1\n");
		ui_model::instance();
		printf("qpa2\n");
	}
} instance;
}
*/

#endif /* __UI_CONFIG_H */

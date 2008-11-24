#ifndef __UI_MODEL_H
#define __UI_MODEL_H

#include <gtk/gtk.h>

#include "ui_config_entry.h"

class ui_model
{
	public:
		static ui_model& instance();

		void clear(void);

		ui_config_entry & add_ui_config_entry(ui_config_entry & parent, ui_config_entry::ui_config_entry_type entry_type, const char *program_name, const char *node_name = NULL, const char *ui_def = NULL);

		GtkTreeStore * getStore() const
		{
			return store;
		}

		ui_config_entry & getRootNode()
		{
			return root_entry;
		}

		ui_config_entry & getNodeByPath(GtkTreePath *path);

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

#endif /* __UI_MODEL_H */

#ifndef __UI_MODEL_H
#define __UI_MODEL_H

#include <gtk/gtk.h>
#include <gtkmm.h>

#include <boost/thread/once.hpp>

#include "ui_config_entry.h"

#include "base/lib/configurator.h"
#include "base/lib/sr/srlib.h"
#include "ui/src/ui.h"

class ui_model
{
public:
	static ui_model& instance();
	static void freeInstance();

	void clear(void);

	ui_config_entry
			& add_ui_config_entry(ui_config_entry & parent, ui_config_entry::ui_config_entry_type entry_type, const char *program_name, const char *node_name =
					NULL, const char *ui_def = NULL);

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

	//! method to manage tab panel visibility
	void show_page(bool visible);

	//! set properties of buttons
	void setMpLoadButton(bool sensitive, bool button_type_is_load);
	void setEdpsLoadButton(bool sensitive, bool button_type_is_load);

	//! load EDP processes
	void loadEdps(void);

	//! slay all configured processes
	void slayAll(void);

	//! set status bar message
	guint set_status(const char *msg);

	//! set UI state notification
	guint set_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion);

	//! set current notebook page
	void set_current_page(gint page_num);

	GObject *getUiGObject(const gchar *name);
	Glib::RefPtr <Glib::Object> getUiObject(const gchar *name);

	//! initialize SR client objects
	void init_sr(void);

	mrrocpp::lib::sr_ecp & getEcpSr(void) const;

	lib::configurator & getConfigurator(void) const;

private:
	static boost::once_flag once;
	static ui_model * pointerToTheSingletonInstance;

	ui_model();
	static void createInstance(void);

	ui_config_entry root_entry;

	// GtkTreeView kinematic_model_with_tool
	GtkTreeStore *store;

	ui_model(ui_model const&); //not defined, not copyable
	ui_model& operator=(ui_model const&); //not defined, not assignable
	~ui_model();

	int tabs_visible;

	GtkBuilder *builder;

	int set_tree_view(void);

	//! old-type .INI configurator
	lib::configurator *interface.config;

	//! SR object for UI
	mrrocpp::lib::sr_ui* ui_report;

	//! SR object for ECP
	mrrocpp::lib::sr_ecp* ecp_report;
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

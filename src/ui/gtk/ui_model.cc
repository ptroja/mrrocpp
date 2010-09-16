#include <gtk/gtk.h>
#include <gtkmm.h>

#include <boost/foreach.hpp>

#include <exception>

#include "ui_model.h"

#include "base/lib/configurator.h"

ui_model * ui_model::pointerToTheSingletonInstance = NULL;

// TODO: rewrite to boost::mutex
boost::once_flag ui_model::once = BOOST_ONCE_INIT;

ui_config_entry & ui_model::getNodeByPath(GtkTreePath *path) {
	gint depth = gtk_tree_path_get_depth(path);
	gint *indices = gtk_tree_path_get_indices(path);

	ui_config_entry *ret = &getRootNode();

	for(int i = 0; i < depth; i++) {
		ret = ret->children[indices[i]];
	}

	return *ret;
}

void ui_model::createInstance(void) {
	assert(!pointerToTheSingletonInstance);
	pointerToTheSingletonInstance = new ui_model;
}

ui_model& ui_model::instance()
{
	// make the signleton initialization in thread-safe manner
	boost::call_once(&createInstance, once);

	return *pointerToTheSingletonInstance;
}

void ui_model::freeInstance()
{
	// TODO: guard with mutex or not...?

	if (pointerToTheSingletonInstance) {
		delete pointerToTheSingletonInstance;
		pointerToTheSingletonInstance = NULL;
	}
}

void ui_model::clear(void)
{
	gtk_tree_store_clear(this->store);

	this->getRootNode().remove_childs();
}

ui_model::ui_model() : tabs_visible(0),
	ui_report(NULL), ecp_report(NULL)
{
	builder = gtk_builder_new();

	GError *error = NULL;
	if (gtk_builder_add_from_file(builder, "interface.xml", &error) == 0) {
		if (error) {
			fprintf (stderr, "Unable to read file interface.xml: %s\n", error->message);
			g_error_free(error);
			exit(-1);
		} else {
			g_assert(0);
		}
	}

	gtk_builder_connect_signals(builder, NULL);

	// create TreeView kinematic_model_with_tool
	store = gtk_tree_store_new(N_COLUMNS, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_BOOLEAN);

	GtkTreeModelFlags flags = gtk_tree_model_get_flags(GTK_TREE_MODEL(store));
	if (!(flags & GTK_TREE_MODEL_ITERS_PERSIST)) {
		g_warn_if_reached();
		g_error("GTK internals changed: GTK_TREE_MODEL_ITERS_PERSIST check failed\n");
	}

	set_tree_view();

	//! initialization
	setMpLoadButton(false, true);
	setEdpsLoadButton(true, true);

	// TODO: rewrite with boost current_path() and basic_path iterators
	gchar *cwd_str = g_get_current_dir();
	std::string cwd(cwd_str);
	g_free(cwd_str);

	cwd += G_DIR_SEPARATOR_S;
	cwd += "..";
	cwd += G_DIR_SEPARATOR_S;

	this->config = new lib::configurator(
			g_get_host_name(),
			cwd,
			"rcsc.ini", lib::UI_SECTION, "");
}

void ui_model::init_sr(void) {
	ui_report = new lib::sr_ui(mrrocpp::lib::UI, "ui", "sr");
	ecp_report = new lib::sr_ecp(mrrocpp::lib::UI, "ui", "sr");

	ui_report->message("UI report");
	ecp_report->message("ecp report");
}

mrrocpp::lib::sr_ecp & ui_model::getEcpSr(void) const {
	return *(this->ecp_report);
}

lib::configurator & ui_model::getConfigurator(void) const {
	return *(this->config);
}

ui_model::~ui_model()
{
	// clear the kinematic_model_with_tool
	this->clear();

	// remove reference to GtkTreeView kinematic_model_with_tool
	g_object_unref(store);
	g_object_unref(G_OBJECT(builder));

	if (ecp_report) {
		// TODO: fix this, bug in messip?
		delete ecp_report;
	}

	if (ui_report) {
		// TODO: fix this, bug in messip?
		delete ui_report;
	}

	if (config) {
		delete config;
	}
}

ui_config_entry & ui_model::add_ui_config_entry(ui_config_entry & parent_entry, ui_config_entry::ui_config_entry_type entry_type, const char *program_name, const char *node_name, const char *ui_def)
{
	ui_config_entry *entry = new ui_config_entry(entry_type, program_name, node_name, ui_def);

	GtkTreeIter child;

	if (parent_entry.type == ui_config_entry::ROOT) {
		gtk_tree_store_append(store, &child, NULL);
	} else {
		GtkTreeIter parent = parent_entry.getTree_iter();
		gtk_tree_store_append(this->store, &child, &parent);
	}

	gtk_tree_store_set(store, &child, NAME_COLUMN, program_name, NODE_NAME_COLUMN, node_name,
			IS_RUNNING_COLUMN, FALSE, -1);

	entry->setTree_iter(child);

	parent_entry.add_child(*entry);

	return *entry;
}

void ui_model::set_current_page(gint page_num) {
	gtk_notebook_set_current_page(
			GTK_NOTEBOOK(ui_model::instance().getUiGObject("notebook1")),
			page_num);
}

void ui_model::show_page(bool visible) {
	if (visible) {
		tabs_visible++;
	} else {
		if (tabs_visible > 0) tabs_visible--;
	}
	//printf("tabs_visible = %d\n", tabs_visible);

	//g_object_set(G_OBJECT(getNotebook()), "visible", tabs_visible ? TRUE : FALSE, NULL);
}

GObject *ui_model::getUiGObject(const gchar *name) {
	return gtk_builder_get_object(this->builder, name);
}

Glib::RefPtr<Glib::Object> ui_model::getUiObject(const gchar *name) {
	GObject *obj = gtk_builder_get_object(this->builder, name);
	g_assert(obj);
	return Glib::wrap(obj);
}

void ui_model::setMpLoadButton (bool sensitive, bool button_type_is_load) {
	Gtk::ToolButton & MpButton = *Glib::wrap(GTK_TOOL_BUTTON(getUiGObject("mpLoadButton")));

	MpButton.set_sensitive(sensitive);
	if (button_type_is_load) {
		MpButton.set_stock_id(Gtk::Stock::CONNECT);
		MpButton.set_label("mp Load");
	} else {
		MpButton.set_stock_id(Gtk::Stock::DISCONNECT);
		MpButton.set_label("mp Unload");
	}
}

void ui_model::setEdpsLoadButton (bool sensitive, bool button_type_is_load) {
	GObject *obj = getUiGObject("edpsLoadButton");
	g_assert (obj);
	Gtk::ToolButton & EcpsButton = *Glib::wrap(GTK_TOOL_BUTTON(obj));

	EcpsButton.set_sensitive(sensitive);
	if (button_type_is_load) {
		EcpsButton.set_stock_id(Gtk::Stock::CONNECT);
		EcpsButton.set_label("All EDP Load");
	} else {
		EcpsButton.set_stock_id(Gtk::Stock::DISCONNECT);
		EcpsButton.set_label("All EDP Unload");
	}
}


guint ui_model::set_status(const char *msg)
{
	GtkStatusbar *statusbar;
	guint context_id;

	statusbar = GTK_STATUSBAR (ui_model::instance().getUiGObject("statusbar"));

	context_id = gtk_statusbar_get_context_id(statusbar, "base message");

	return gtk_statusbar_push(statusbar, context_id, msg);
}

guint ui_model::set_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion)
{
	GtkStatusbar *statusbar;
	guint context_id;

	statusbar = GTK_STATUSBAR (ui_model::instance().getUiGObject("notifybar"));

	context_id = gtk_statusbar_get_context_id(statusbar, "base message");

	const char *msg;

	switch(new_notifacion) {
		case UI_N_STARTING: msg = "STARTING"; break;
		case UI_N_READY: msg = "READY"; break;
		case UI_N_BUSY: msg = "BUSY"; break;
		case UI_N_EXITING: msg = "EXITING"; break;
		case UI_N_COMMUNICATION: msg = "COMMUNICATION"; break;
		case UI_N_PROCESS_CREATION: msg = "PROCESS CREATION"; break;
		case UI_N_SYNCHRONISATION: msg = "SYNCHRONISATION"; break;
		default: msg = "unkonwn"; break;
	}

	return gtk_statusbar_push(statusbar, context_id, msg);
}

int ui_model::set_tree_view(void)
{
	GtkTreeView *tree = GTK_TREE_VIEW (getUiGObject("process_treeview"));
	gtk_tree_view_set_model(tree, GTK_TREE_MODEL(getStore()));

	GtkCellRenderer *renderer;
	GtkTreeViewColumn *column;

	renderer = gtk_cell_renderer_text_new();

	column = gtk_tree_view_column_new_with_attributes("Program name", renderer, "markup",
			ui_model::NAME_COLUMN, (void *) NULL);
	gtk_tree_view_append_column(tree, column);

	column = gtk_tree_view_column_new_with_attributes("Node name", renderer, "text",
			ui_model::NODE_NAME_COLUMN, (void *) NULL);
	gtk_tree_view_append_column(tree, column);

	renderer = gtk_cell_renderer_toggle_new();
	column = gtk_tree_view_column_new_with_attributes("Running", renderer, "active",
			ui_model::IS_RUNNING_COLUMN, (void *) NULL);
	gtk_tree_view_append_column(tree, column);

	return 0;
}

void ui_model::loadEdps(void) {
        using namespace std;

	ui_config_entry::childrens_t edps = ui_model::instance().getRootNode().getChildByType(ui_config_entry::EDP);

	BOOST_FOREACH(ui_config_entry * entry, edps) {
	  std::cout << entry->program_name << "@" << entry->node_name << std::endl;
	  string cmd = entry->program_name + " soplica" + " /home/konradb3/mrrocpp/" + " xml/rcsc.xml"+ " [" + entry->program_name + "]" + " ";
	  system(cmd.c_str());

	}
}

void ui_model::slayAll(void)
{
	/*
	 * TODO: all the slay&killall command should be added to list,
	 * then called clear() to gently unload all nodes and finally
	 * execute commands from the list
	 */
	using namespace std;

	const string RSHcommand = "rsh ";
	const string slayCommand = " killall -9 ";

	ui_config_entry::childrens_t edps(ui_model::instance().getRootNode().getChildByType(ui_config_entry::EDP));
	ui_config_entry::childrens_t ecps(ui_model::instance().getRootNode().getChildByType(ui_config_entry::ECP));
	ui_config_entry::childrens_t mp(ui_model::instance().getRootNode().getChildByType(ui_config_entry::MP));

	BOOST_FOREACH(ui_config_entry * entry, edps) {
		string command = slayCommand + entry->program_name;
		cout << command << endl;
		system(command.c_str());
	}

	BOOST_FOREACH(ui_config_entry * entry, ecps) {
		string command = slayCommand + entry->program_name;
		cout << command << endl;
		system(command.c_str());
	}

	BOOST_FOREACH(ui_config_entry * entry, mp) {
		string command = slayCommand + entry->program_name;
		cout << command << endl;
//		system(command.c_str());
	}

	this->clear();
}

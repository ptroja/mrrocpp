#include <gtk/gtk.h>
#include <gtkmm.h>

#include "ui_model.h"

ui_config_entry & ui_model::getNodeByPath(GtkTreePath *path) {
	gint depth = gtk_tree_path_get_depth(path);
	gint *indices = gtk_tree_path_get_indices(path);

	ui_config_entry *ret = &getRootNode();

	for(int i = 0; i < depth; i++) {
		ret = ret->children[indices[i]];
	};

	return *ret;
}

ui_model& ui_model::instance()
{
	static ui_model * pointerToTheSingletonInstance = new ui_model;
	return *pointerToTheSingletonInstance;
}

void ui_model::clear(void)
{
	gtk_tree_store_clear(this->store);

	this->getRootNode().remove_childs();
}

ui_model::ui_model() : tabs_visible(0)
{
	builder = gtk_builder_new();

	GError *error = NULL;
	if (gtk_builder_add_from_file(builder, "ui.xml", &error) == 0) {
		if (error) {
			fprintf (stderr, "Unable to read file ui.xml: %s\n", error->message);
			g_error_free(error);
			exit(-1);
		} else {
			g_assert(0);
		}
	}

	gtk_builder_connect_signals(builder, NULL);

	// create TreeView model
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
}

ui_model::~ui_model()
{
	// clear the model
	this->clear();

	// remove reference to GtkTreeView model
	g_object_unref(store);
	g_object_unref(G_OBJECT(builder));
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
	Gtk::ToolButton & MpButton = *Glib::wrap(GTK_TOOL_BUTTON(getUiGObject("MpLoadButton")));

	MpButton.set_sensitive(sensitive);
	if (button_type_is_load) {
		MpButton.set_stock_id(Gtk::Stock::CONNECT);
		MpButton.set_label("MP Load");
	} else {
		MpButton.set_stock_id(Gtk::Stock::DISCONNECT);
		MpButton.set_label("MP Unload");
	}
}

void ui_model::setEdpsLoadButton (bool sensitive, bool button_type_is_load) {
	GObject *obj = getUiGObject("EdpsLoadButton");
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

int ui_model::set_tree_view(void)
{
	GtkTreeView *tree = GTK_TREE_VIEW (getUiGObject("process_treeview"));
	gtk_tree_view_set_model(tree, GTK_TREE_MODEL(getStore()));

	GtkCellRenderer *renderer;
	GtkTreeViewColumn *column;

	renderer = gtk_cell_renderer_text_new();

	column = gtk_tree_view_column_new_with_attributes("Program name", renderer, "markup",
			ui_model::NAME_COLUMN, NULL);
	gtk_tree_view_append_column(tree, column);

	column = gtk_tree_view_column_new_with_attributes("Node name", renderer, "text",
			ui_model::NODE_NAME_COLUMN, NULL);
	gtk_tree_view_append_column(tree, column);

	renderer = gtk_cell_renderer_toggle_new();
	column = gtk_tree_view_column_new_with_attributes("Running", renderer, "active",
			ui_model::IS_RUNNING_COLUMN, NULL);
	gtk_tree_view_append_column(tree, column);

	return 0;
}

void ui_model::loadEdps(void) {
	std::vector <ui_config_entry *> edps = ui_model::instance().getRootNode().getChildByType(ui_config_entry::EDP);

	for(std::vector<ui_config_entry *>::iterator Iter = edps.begin(); Iter != edps.end(); Iter++) {
		std::cout << (*Iter)->program_name << "@" << (*Iter)->node_name << std::endl;
	}
}

#include "ui_model.h"

/*void ui_config_entry::add_child(ui_config_entry & child)  {
	childs.insert(childs.begin(), child);
}*/

ui_config_entry::ui_config_entry() : type(ROOT) {
}

ui_config_entry::ui_config_entry(ui_config_entry_type _type, const char *program, const char *node) : program_name(program), node_name(node ? node : ""), type(_type) {
}

ui_config_entry & ui_model::add_ui_config_entry(ui_config_entry & parent_entry, ui_config_entry::ui_config_entry_type entry_type, const char *program_name, const char *node_name)
{
	ui_config_entry *entry = new ui_config_entry(entry_type, program_name, node_name);

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

	return *entry;
}

ui_model& ui_model::instance()
{
	static ui_model * pointerToTheSingletonInstance = new ui_model;
	return *pointerToTheSingletonInstance;
}

void ui_model::clear(void)
{
	// TODO: parse model and call delete();

	gtk_tree_store_clear(store);
}

ui_model::ui_model()
{
	// create TreeView model
	store = gtk_tree_store_new(N_COLUMNS, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_BOOLEAN);

	GtkTreeModelFlags flags = gtk_tree_model_get_flags(GTK_TREE_MODEL(store));
	if (!(flags & GTK_TREE_MODEL_ITERS_PERSIST)) {
		g_error("GTK internals changed: GTK_TREE_MODEL_ITERS_PERSIST check failed, %s@%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
	}
}

ui_model::~ui_model()
{
	// clear the model
	this->clear();

	// remove reference to GtkTreeView model
	g_object_unref(store);
}

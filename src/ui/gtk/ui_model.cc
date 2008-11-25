#include <gtk/gtk.h>

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

extern "C" {
	gint my_popup_handler(GtkWidget *widget, GdkEvent *event);
}

gint my_popup_handler(GtkWidget *widget, GdkEventButton *event, gpointer user_data)
{
	GtkMenu *menu;
	GdkEventButton *event_button;

	g_return_val_if_fail (widget != NULL, FALSE);
	g_return_val_if_fail (GTK_IS_MENU (widget), FALSE);
	g_return_val_if_fail (event != NULL, FALSE);

	/* The "widget" is the menu that was supplied when
	 * g_signal_connect_swapped() was called.
	 */
	menu = GTK_MENU (widget);

	if (event->type == GDK_BUTTON_PRESS)
	{
		event_button = (GdkEventButton *) event;
		if (event_button->button == 3)
		{
			gtk_menu_popup (menu, NULL, NULL, NULL, NULL,
					event_button->button, event_button->time);
			return TRUE;
		}
	}

	return FALSE;
}

ui_model::ui_model()
{
	// create TreeView model
	store = gtk_tree_store_new(N_COLUMNS, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_BOOLEAN);

	GtkTreeModelFlags flags = gtk_tree_model_get_flags(GTK_TREE_MODEL(store));
	if (!(flags & GTK_TREE_MODEL_ITERS_PERSIST)) {
		g_warn_if_reached();
		g_error("GTK internals changed: GTK_TREE_MODEL_ITERS_PERSIST check failed\n");
	}

	//g_signal_connect_swapped(store, "button-press-event", G_CALLBACK(my_popup_handler), NULL);
}

ui_model::~ui_model()
{
	// clear the model
	this->clear();

	// remove reference to GtkTreeView model
	g_object_unref(store);
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

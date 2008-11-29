#include <gtk/gtk.h>
#include <gtk/gtktreemodel.h>

#include <gtkmm.h>

#include "configurator.h"
#include "ui_model.h"
#include "ui_utils.h"

GtkNotebook *getNotebook(void) {
	return GTK_NOTEBOOK (ui_model::instance().getUiObject("notebook1"));
}

guint set_status(const char *msg)
{
	GtkStatusbar *statusbar;
	guint context_id;

	statusbar = GTK_STATUSBAR (ui_model::instance().getUiObject("statusbar"));

	context_id = gtk_statusbar_get_context_id(statusbar, "base message");

	return gtk_statusbar_push(statusbar, context_id, msg);
}

int set_tree_view(void)
{
	GtkTreeView *tree = GTK_TREE_VIEW (ui_model::instance().getUiObject("process_treeview"));
	gtk_tree_view_set_model(tree, GTK_TREE_MODEL(ui_model::instance().getStore()));

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

extern "C" {

	void on_window_destroy(GtkObject *object, gpointer user_data)
	{
		gtk_main_quit();
	}

	void on_open_file_activate(GtkObject *object, gpointer user_data)
	{
		GtkWidget *dialog;

		dialog = gtk_file_chooser_dialog_new("Open File", NULL, //parent_window,
				GTK_FILE_CHOOSER_ACTION_OPEN, GTK_STOCK_CANCEL,
				GTK_RESPONSE_CANCEL, GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT, NULL);

		GtkFileFilter *filter = gtk_file_filter_new();
		gtk_file_filter_add_pattern(filter, "*.xml");
		gtk_file_filter_set_name(filter, "XML files");
		gtk_file_chooser_set_filter(GTK_FILE_CHOOSER(dialog), filter);

		gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(dialog), config->getConfig_dir());

		if (gtk_dialog_run(GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT) {
			char *filename;

			filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER (dialog));
			config->open_config_file(filename);
			g_free(filename);
		}

		gtk_widget_destroy(dialog);
	}

	void on_process_treeview_row_activated(GtkTreeView *treeview, GtkTreePath *path, GtkTreeViewColumn *col, gpointer userdata)
	{
		ui_config_entry &entry = ui_model::instance().getNodeByPath(path);
		entry.show_page(TRUE);
	}

} /* extern "C" */

gchar **config_files;

static GOptionEntry entries[] =
{
  { G_OPTION_REMAINING, 0, 0, G_OPTION_ARG_FILENAME_ARRAY, &config_files, "config file", NULL },
  { NULL }
};

int main(int argc, char *argv[])
{
	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- MRROC++ User Interface");
	g_option_context_add_main_entries (context, entries, NULL);
	g_option_context_add_group (context, gtk_get_option_group (TRUE));
	if (!g_option_context_parse (context, &argc, &argv, &error)) {
		g_print ("option parsing failed: %s\n", error->message);
		exit (1);
	}
	g_option_context_free(context);

	//! Gtk::Main object initialized for use GTKMM in process tabs
	Gtk::Main kit(argc, argv);

	config = new configurator();

	set_status("MRROC++ - the best robotic framework ever");

	set_tree_view();

	gtk_widget_show(GTK_WIDGET(ui_model::instance().getUiObject("window")));

	if(config_files && *config_files) {
		std::string config_file = config->getConfig_dir();
		config_file += "/";
		config_file += (*config_files);
		config->open_config_file(config_file.c_str());
	}

	gtk_main();

	return 0;
}

extern "C" {
	void mp_load_clicked(GtkToolButton *toolbutton,
            gpointer       user_data) {
		Gtk::ToolButton & LoadButton = *Glib::wrap(toolbutton);

		LoadButton.set_label("MP Unload");
		LoadButton.set_stock_id(Gtk::Stock::DISCONNECT);
	}

	void edp_load_clicked(GtkToolButton *toolbutton,
            gpointer       user_data) {
		Gtk::ToolButton & LoadButton = *Glib::wrap(toolbutton);

		LoadButton.set_label("EDP Unload");
		LoadButton.set_stock_id(Gtk::Stock::DISCONNECT);
	}

}

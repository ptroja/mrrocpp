#include <gtk/gtk.h>
#include <gtk/gtktreemodel.h>

#include "configurator.h"
#include "ui_model.h"
#include "ui_utils.h"

// TODO: this should'n be global
static GtkBuilder *builder;

GtkNotebook *getNotebook(void) {
	return GTK_NOTEBOOK (gtk_builder_get_object (builder, "notebook1"));
}

guint set_status(const char *msg)
{
	GtkStatusbar *statusbar;
	guint context_id;

	statusbar = GTK_STATUSBAR (gtk_builder_get_object (builder, "statusbar"));

	context_id = gtk_statusbar_get_context_id(statusbar, "base message");

	return gtk_statusbar_push(statusbar, context_id, msg);
}

int set_tree_view(void)
{
	GtkTreeView *tree = GTK_TREE_VIEW (gtk_builder_get_object(builder, "process_treeview"));
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
//		printf("%s\n", gtk_tree_path_to_string(path));

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
	//g_option_context_free(context);

	config = new configurator();

	builder = gtk_builder_new();
	gtk_builder_add_from_file(builder, "ui.xml", NULL);

	gtk_builder_connect_signals(builder, NULL);

	set_status("MRROC++ - the best robotic framework ever");

	set_tree_view();

	gtk_widget_show(GTK_WIDGET (gtk_builder_get_object (builder, "window")));

	if(config_files && *config_files) {
		std::string config_file = config->getConfig_dir();
		config_file += "/";
		config_file += (*config_files);
		config->open_config_file(config_file.c_str());
	}

	gtk_main();

	g_object_unref(G_OBJECT(builder));

	return 0;
}

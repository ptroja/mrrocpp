/*
 First run tutorial.glade through gtk-builder-convert with this command:
 gtk-builder-convert tutorial.glade tutorial.xml

 Then save this file as main.c and compile it using this command
 (those are backticks, not single quotes):
 gcc -Wall -g -o tutorial main.c `pkg-config --cflags --libs gtk+-2.0` -export-dynamic

 Then execute it using:
 ./tutorial
 */
#include <gtk/gtk.h>

#include "configurator.h"

GtkBuilder *builder;

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
	GtkTreeView *tree = GTK_TREE_VIEW (gtk_builder_get_object(builder, "treeview1"));
	gtk_tree_view_set_model(tree, GTK_TREE_MODEL(config->getStore()));

	GtkCellRenderer *renderer;
	GtkTreeViewColumn *column;

	renderer = gtk_cell_renderer_text_new();

	column = gtk_tree_view_column_new_with_attributes("Program name", renderer, "markup",
			configurator::NAME_COLUMN, NULL);
	gtk_tree_view_append_column(tree, column);

	column = gtk_tree_view_column_new_with_attributes("Node name", renderer, "text",
			configurator::NODE_NAME_COLUMN, NULL);
	gtk_tree_view_append_column(tree, column);

	renderer = gtk_cell_renderer_toggle_new();
	column = gtk_tree_view_column_new_with_attributes("Running", renderer, "active",
			configurator::IS_RUNNING_COLUMN, NULL);
	gtk_tree_view_append_column(tree, column);
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

} /* extern "C" */

int main(int argc, char *argv[])
{
	GtkWidget *window;

	gtk_init(&argc, &argv);

	config = new configurator();

	builder = gtk_builder_new();
	gtk_builder_add_from_file(builder, "ui.xml", NULL);
	window = GTK_WIDGET (gtk_builder_get_object (builder, "window"));

	gtk_builder_connect_signals(builder, NULL);

	set_status("MRROC++ - the best robotic framework ever");
	/*

	 {
	 GtkNotebook *notebook = GTK_NOTEBOOK (gtk_builder_get_object(builder, "notebook1"));
	 GtkWidget *tab = GTK_WIDGET (gtk_builder_get_object(builder, "hbuttonbox1"));

	 gtk_widget_unparent(tab);
	 gtk_notebook_append_page(notebook, tab, NULL);
	 }

	 */

	set_tree_view();

	gtk_widget_show(window);
	gtk_main();

	g_object_unref(G_OBJECT(builder));

	return 0;
}

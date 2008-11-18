#include <gtk/gtk.h>
#include <gtk/gtktreemodel.h>

#include "configurator.h"
#include "ui_model.h"

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
		gint depth = gtk_tree_path_get_depth(path);
		gint *indices = gtk_tree_path_get_indices(path);

		while(depth--) {
			printf("%d:", *indices++);
		}
		printf("\n");
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

	for(int i = 0; i < 1; i++)
	{
		GtkWindow *tab_window = GTK_WINDOW (gtk_builder_get_object(builder, "window1"));

		GtkWidget *content = gtk_bin_get_child(GTK_BIN(tab_window));
		gtk_widget_unparent(content);

		GtkHBox *hbox = GTK_HBOX(gtk_hbox_new(FALSE, 5));
		GtkImage *tabicon = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_EXECUTE, GTK_ICON_SIZE_BUTTON));
		GtkLabel *tablabel = GTK_LABEL(gtk_label_new("lalalabel"));
		GtkImage *tabcloseicon = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_CLOSE, GTK_ICON_SIZE_MENU));
		gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(tabicon));
		gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(tablabel));
		gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(tabcloseicon));
		gtk_widget_show_all(GTK_WIDGET(hbox));

		GtkNotebook *notebook = GTK_NOTEBOOK (gtk_builder_get_object(builder, "notebook1"));

		gtk_notebook_append_page(notebook, content, GTK_WIDGET(hbox));
	}

	set_tree_view();

	gtk_widget_show(window);
	gtk_main();

	g_object_unref(G_OBJECT(builder));

	return 0;
}

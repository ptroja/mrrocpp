#include <gtk/gtk.h>
#include <gtk/gtktreemodel.h>

#include <gtkmm.h>

#include "configurator.h"
#include "ui_model.h"
#include "sr_console.h"

GtkNotebook *getNotebook(void) {
	return GTK_NOTEBOOK (ui_model::instance().getUiGObject("notebook1"));
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

	if (!g_thread_supported ()) g_thread_init (NULL);

	config = new configurator();

	{
		GError *err;

		GThread *sr = g_thread_create(sr_thread, NULL, true, &err);
		if (sr == NULL) {
			fprintf(stderr, "g_thread_create(): %s\n", err->message);
			return -1;
		}
	}

	ui_model::instance().set_status("MRROC++ - the best robotic framework ever");
	ui_model::instance().setEdpsLoadButton(false, true);
	ui_model::instance().setMpLoadButton(false, true);

	gtk_widget_show(GTK_WIDGET(ui_model::instance().getUiGObject("window")));

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

		g_print("host %s\n", g_get_host_name());
		ui_model::instance().setMpLoadButton(true, false);
	}

	void edp_load_clicked(GtkToolButton *toolbutton,
            gpointer       user_data) {

		ui_model::instance().setEdpsLoadButton(true, false);
		ui_model::instance().loadEdps();
	}

}

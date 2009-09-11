#include <gtk/gtk.h>
#include <gtk/gtktreemodel.h>

#include <gtkmm.h>

#include "xmlconfigurator.h"
#include "ui_model.h"
#include "sr_console.h"
#include "comm_thread.h"

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

		gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(dialog), xmlconfig->getConfig_dir());

		if (gtk_dialog_run(GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT) {
			char *filename;

			filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER (dialog));
			xmlconfig->open_config_file(filename);
			g_free(filename);
		}

		gtk_widget_destroy(dialog);
	}

	void on_process_treeview_row_activated(GtkTreeView *treeview, GtkTreePath *path, GtkTreeViewColumn *col, gpointer userdata)
	{
		ui_config_entry &entry = ui_model::instance().getNodeByPath(path);
		entry.show_page(TRUE);
	}

	void on_SlayAllButton_clicked(GtkButton *button, gpointer user_data) {
		ui_model::instance().slayAll();
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
	// initialize multi-threading in GTK/GDK
	if (!g_thread_supported ()) g_thread_init (NULL);
	gdk_threads_init();
	gdk_threads_enter();

	// common GLib error pointer
	GError *error = NULL;

	// parse command line options
	GOptionContext *context;
	context = g_option_context_new ("- MRROC++ User Interface");
	g_option_context_add_main_entries (context, entries, NULL);
	g_option_context_add_group (context, gtk_get_option_group (TRUE));

	// this calls gtk_init()
	if (!g_option_context_parse (context, &argc, &argv, &error)) {
		g_print ("option parsing failed: %s\n", error->message);
		exit (1);
	}
	g_option_context_free(context);

	//! Gtk::Main object initialized for use GTKMM in process tabs
	Gtk::Main kit(argc, argv);

	xmlconfig = new xmlconfigurator();

	// create SR thread
	GThread *sr_t = g_thread_create(sr_thread, NULL, false, &error);
	if (sr_t == NULL) {
		fprintf(stderr, "g_thread_create(): %s\n", error->message);
		return -1;
	}

	// create common ECP->UI communication thread
	GThread *comm_t = g_thread_create(comm_thread, NULL, false, &error);
	if (comm_t == NULL) {
		fprintf(stderr, "g_thread_create(): %s\n", error->message);
		return -1;
	}

	ui_model::instance().set_status("MRROC++ - the best robotic framework ever");
	ui_model::instance().setEdpsLoadButton(false, true);
	ui_model::instance().setMpLoadButton(false, true);

	gtk_widget_show(GTK_WIDGET(ui_model::instance().getUiGObject("window")));

	if(config_files && *config_files) {
		std::string config_file = xmlconfig->getConfig_dir();
		config_file += "/";
		config_file += (*config_files);
		xmlconfig->open_config_file(config_file.c_str());
	}

	// initialize client SR objects
	ui_model::instance().init_sr();

//	GtkDialog *input = GTK_DIALOG(ui_model::instance().getUiGObject("window-input-number"));
//	gtk_widget_show_all(GTK_WIDGET(input));


	try {
	gtk_main();
	}
	catch (...) {  /* Dla zewnetrznej petli try*/ \
		/* Wylapywanie niezdefiniowanych bledow*/ \
		/* Komunikat o bledzie wysylamy do SR (?) */ \
		fprintf(stderr, "unidentified error in UI\n"); \
	}

	printf("main() exiting...\n");

	ui_model::freeInstance();

	gdk_threads_leave();

	return 0;
}

extern "C" {
	void mp_load_clicked(GtkToolButton *toolbutton, gpointer user_data)	{
		ui_model::instance().setMpLoadButton(true, false);
	}

	void edp_load_clicked(GtkToolButton *toolbutton, gpointer user_data) {
		ui_model::instance().setEdpsLoadButton(true, false);
		ui_model::instance().loadEdps();
	}

}

// FIXME: moved from proto.h for linux compatibility
int set_ui_state_notification ( UI_NOTIFICATION_STATE_ENUM new_notifacion ){
	return ui_model::instance().set_state_notification(new_notifacion);
}

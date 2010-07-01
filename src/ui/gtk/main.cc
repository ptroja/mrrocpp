#include <gtk/gtk.h>
#include <gtk/gtktreemodel.h>

#include <stdlib.h>

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
		ui_model::freeInstance();
		gtk_main_quit();
	}

	void on_open_file_activate(GtkObject *object, gpointer user_data)
	{
		GtkWidget *dialog;

		dialog = gtk_file_chooser_dialog_new(
				"Open File", // title
				NULL, // parent_window,
				GTK_FILE_CHOOSER_ACTION_OPEN, // action
				// stock/respone ID pairs, ending with NULL
				GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
				GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT,
				(void *) NULL);

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

gchar **ui.config_files;

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

	// TODO: this should be singleton
	xmlconfig = new xmlconfigurator();

	// create SR thread
	GThread *sr_tid = g_thread_create(sr_thread, NULL, FALSE, &error);
	if (sr_tid == NULL) {
		fprintf(stderr, "g_thread_create(): %s\n", error->message);
		return -1;
	}

	// create common ECP->UI communication thread
	GThread *comm_tid = g_thread_create(comm_thread, NULL, FALSE, &error);
	if (comm_tid == NULL) {
		fprintf(stderr, "g_thread_create(): %s\n", error->message);
		return -1;
	}

	ui_model::instance().set_status("MRROC++ - the best robotic framework ever");
	ui_model::instance().setEdpsLoadButton(false, true);
	ui_model::instance().setMpLoadButton(false, true);

	gtk_widget_show(GTK_WIDGET(ui_model::instance().getUiGObject("window")));

	if(config_files && *ui.config_files) {
		std::string config_file = xmlconfig->getConfig_dir();
		config_file += "/";
		config_file += (*ui.config_files);
		xmlconfig->open_config_file(config_file.c_str());
	}

	// initialize client SR objects
	ui_model::instance().init_sr();

//	GtkDialog *input = GTK_DIALOG(ui_model::instance().getUiGObject("window-input-number"));
//	gtk_widget_show_all(GTK_WIDGET(input));

	// call the main event loop
	gtk_main();

	// we are going back home
	printf("UI: exiting...\n");

	// clean up the staff
	ui_model::freeInstance();
	delete xmlconfig;

	// leave the critical GTK section
	gdk_threads_leave();

	return 0;
}

extern "C" {
	void on_MpLoadButton_clicked(GtkToolButton *toolbutton, gpointer user_data)	{
		ui_model::instance().setMpLoadButton(true, false);
	}

	void on_EdpsLoadButton_clicked(GtkToolButton *toolbutton, gpointer user_data) {
		ui_model::instance().setEdpsLoadButton(true, false);
		ui_model::instance().loadEdps();
	}

}

// FIXME: moved from proto.h for linux compatibility
int set_ui_state_notification ( UI_NOTIFICATION_STATE_ENUM new_notifacion ){
	return ui_model::instance().set_state_notification(new_notifacion);
}

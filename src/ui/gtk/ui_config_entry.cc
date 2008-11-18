#include "ui_config_entry.h"
#include "ui_model.h"

bool ui_config_entry::is_empty(void) {
	return children.empty();
}

void ui_config_entry::add_child(ui_config_entry & child)  {
	children.push_back(&child);
}

void ui_config_entry::remove_childs(void)  {

	std::vector<ui_config_entry *>::iterator Iter;

	for (Iter = children.begin(); Iter != children.end();) {

		if ((*Iter)->is_empty() == false) {
			(*Iter)->remove_childs();
		}

		ui_config_entry *empty_child = (*Iter);
		Iter++;
		delete empty_child;
	}
	children.clear();
}

ui_config_entry::ui_config_entry() : type(ROOT), builder(NULL), window(NULL), module(NULL) {
}

ui_config_entry::ui_config_entry(ui_config_entry_type _type, const char *program, const char *node, const char *ui_def) : program_name(program), node_name(node ? node : ""), type(_type) {
	if (ui_def) {
		builder = gtk_builder_new();
		g_assert(builder);

		GError *err = NULL;
		if (gtk_builder_add_from_file(builder, ui_def, &err) == 0) {
			fprintf (stderr, "Unable to read file %s: %s\n", ui_def, err->message);
			g_error_free (err);

			// TODO: throw(...)
		}

		window = GTK_WIDGET (gtk_builder_get_object (builder, "window"));

		if (!window) {
			fprintf(stderr, "Unable to get \"window\" widget from %s file", ui_def);
			// TODO: throw(...)
		}

		// remove filename extension, add local directory prefix
		std::string ui_lib = std::string(ui_def);
		ui_lib.erase(ui_lib.find_last_of('.'));
		ui_lib.insert(0, "./");

		module = g_module_open(ui_lib.c_str(), (GModuleFlags) 0);

		if(module) {
			gtk_builder_connect_signals(builder, this);
		} else {
			g_warning("failed to open module %s.%s\n", ui_lib.c_str(), G_MODULE_SUFFIX );
		}

		page_visible = false;

	} else {
		builder = NULL;
	}
}

ui_config_entry::~ui_config_entry() {
	if(builder) {
		g_object_unref(G_OBJECT(builder));
	}

	if (module) {
		if(!g_module_close(module)) {
			const char *module_name = (char *) g_module_name(module);
			fprintf(stderr, "error closing module %s", module_name);
		}
	}
}
extern "C" {
	gboolean tabcloseicon_event_cb(GtkWidget *widget, GdkEventButton *event, gpointer userdata) {
		printf("%s@%s:%d widget.name=%s ", __FUNCTION__, __FILE__, __LINE__, widget->name);

		//gtk_container_get_children

		switch(event->type) {
			case GDK_BUTTON_PRESS:
				printf("GDK_BUTTON_PRESS");
				break;
			case GDK_BUTTON_RELEASE:
				printf("GDK_BUTTON_RELEASE");
				break;
			default:
				break;
		}
		printf("\n");
		return TRUE;
	}
}

void ui_config_entry::show_page(GtkNotebook *notebook) {

	if (page_visible) return;
	if (!this->window) return;

	GtkWidget *content = gtk_bin_get_child(GTK_BIN(this->window));
	gtk_widget_unparent(content);

	GtkHBox *hbox = GTK_HBOX(gtk_hbox_new(FALSE, 5));
	GtkImage *tabicon = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_EXECUTE, GTK_ICON_SIZE_BUTTON));
	GtkLabel *tablabel = GTK_LABEL(gtk_label_new(this->program_name.c_str()));
	GtkEventBox *event_box = GTK_EVENT_BOX(gtk_event_box_new());
	GtkImage *tabcloseicon = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_CLOSE, GTK_ICON_SIZE_MENU));

	gtk_widget_set_name (GTK_WIDGET(event_box), "event_box");
	gtk_widget_set_name (GTK_WIDGET(tabcloseicon), "tabcloseicon");

	gtk_container_add (GTK_CONTAINER (event_box), GTK_WIDGET(tabcloseicon));
	gtk_event_box_set_visible_window(event_box, FALSE);

	gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(tabicon));
	gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(tablabel));
	gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(event_box/*tabcloseicon*/));
	gtk_widget_show_all(GTK_WIDGET(hbox));

	gtk_notebook_append_page(notebook, content, GTK_WIDGET(hbox));

	g_signal_connect(G_OBJECT (event_box), "button-press-event", G_CALLBACK(tabcloseicon_event_cb), this);
	g_signal_connect(G_OBJECT (event_box), "button-release-event", G_CALLBACK(tabcloseicon_event_cb), this);

	page_visible = true;
}

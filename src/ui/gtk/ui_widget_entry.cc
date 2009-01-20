#include "ui_widget_entry.h"
#include "ui_model.h"

#include <dlfcn.h>

static void print_gobject(gpointer data, gpointer user_data) {
	GObject *obj = (GObject *) data;
	printf(" object\n");
}

void ui_widget_entry::ListBuilderObjects(void) {

	g_assert(this->builder);

	GSList *list = gtk_builder_get_objects(this->builder);

	if (!list) {
		std::cerr << "gtk_builder_get_objects() failed" << std::endl;
		return;
	}

	g_slist_foreach(list, print_gobject, NULL);

	g_slist_free(list);
}

void ui_widget_entry::copy(ui_widget_entry &entry)
{
	std::cout << "copying entries, please wait...." << std::endl;
	GtkBuilder & thisBuilder = (entry.getBuilder());
}

ui_widget_entry::ui_widget_entry(const char *ui_def)  {

	if (!ui_def) {
		builder = NULL;
		module = NULL;
		g_warn_if_reached();
		return;
	}

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
	g_assert(ui_def);
	std::string ui_lib = std::string(ui_def);
	ui_lib.erase(ui_lib.find_last_of('.'));
	ui_lib.insert(0, "./");
	ui_lib.append(".");
	ui_lib.append(G_MODULE_SUFFIX);

	module = g_module_open(ui_lib.c_str(), (GModuleFlags) G_MODULE_BIND_LAZY);

	if(module) {
		gtk_builder_connect_signals(builder, this);

		gpointer symbol;
		if (g_module_symbol(module, "ui_widget_init", &symbol)) {

			typedef void (*ui_widget_init_t)(ui_widget_entry &entry);

			ui_widget_init_t ui_widget_init = (ui_widget_init_t) symbol;

			ui_widget_init(*this);
		} else {
			g_warning("failed to call widget %s.%s init function\n", ui_lib.c_str(), G_MODULE_SUFFIX );
		}
	} else {
		g_warning("failed to open widget %s\n", ui_lib.c_str());

		//! just to debug
		void *handle = dlopen(ui_lib.c_str(), RTLD_LAZY|RTLD_GLOBAL);
		if (!handle) {
			fprintf(stderr, "dlopen(): %s\n", dlerror());
		} else {
			printf("dlopen %s OK\n", ui_lib.c_str());
			dlclose(handle);
		}
	}
}

ui_widget_entry::~ui_widget_entry()
{
	if (module) {
		gpointer symbol;
		if (g_module_symbol(module, "ui_widget_unload", &symbol)) {

			typedef void (*ui_widget_unload_t)(void);

			ui_widget_unload_t ui_widget_unload = (ui_widget_unload_t) symbol;

			ui_widget_unload();
		}

		if(!g_module_close(module)) {
			const char *module_name = (char *) g_module_name(module);
			fprintf(stderr, "error closing module %s", module_name);
		}
	}
}

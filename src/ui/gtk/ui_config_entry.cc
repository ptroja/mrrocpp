#include "ui_config_entry.h"
#include "ui_model.h"

#include <dlfcn.h>

extern "C" {
	void tabcloseicon_event_cb(GtkButton *button, gpointer userdata) {

		ui_config_entry & entry = *(ui_config_entry *) userdata;
		entry.show_page(FALSE);
	}
}

GtkNotebook *ui_config_entry::getNotebook(void) {
	return GTK_NOTEBOOK(ui_model::instance().getUiGObject("notebook1"));
}

GtkTreeIter & ui_config_entry::getTree_iter() const {
	return *((GtkTreeIter*)&tree_iter);
}

void ui_config_entry::setTree_iter(GtkTreeIter & _tree_iter) {
	this->tree_iter = (_tree_iter);
}

void ui_config_entry::show_page(bool visible) {
	if (!content) return;
	g_object_set(this->content, "visible", visible, NULL);

	ui_model::instance().show_page(visible);

	gint pageIndex = gtk_notebook_page_num(GTK_NOTEBOOK(ui_model::instance().getUiGObject("notebook1")), this->content);
	if (pageIndex >= 0) {
		ui_model::instance().set_current_page(pageIndex);
	}

//	if (visible) {
//		gtk_notebook_append_page(getNotebook(), this->content, GTK_WIDGET(this->hbox));
//		gtk_notebook_set_tab_reorderable(getNotebook(), this->content, TRUE);
//	} else {
//		int page_no = gtk_notebook_page_num(getNotebook(), this->content);
//		printf("page_no = %d\n", page_no);
//		gtk_notebook_remove_page(getNotebook(), page_no);
//	}
}

int ui_config_entry::childrens(void) {
	return children.size();
}

void ui_config_entry::add_child(ui_config_entry & child)  {
	children.push_back(&child);
//	std::cout << program_name << ".size() = " << children.size() << std::endl;
}

void ui_config_entry::remove_childs(void)  {

	for (childrens_t::iterator Iter = children.begin(); Iter != children.end(); Iter++) {

		if ((*Iter)->childrens()) {
			(*Iter)->remove_childs();
		}

		ui_config_entry *empty_child = (*Iter);
		delete empty_child;
	}
	children.clear();
}

ui_config_entry::ui_config_entry() : type(ROOT), builder(NULL), window(NULL), module(NULL) {
}

ui_config_entry::ui_config_entry(ui_config_entry_type _type, const char *program, const char *node, const char *ui_def)
:
	program_name(program ? program : "MISSING PROGRAM NAME"),
	node_name(node ? node : "MISSING NODE NAME"),
	type(_type) {

	if (!ui_def) {
		builder = NULL;
		content = NULL;
		module = NULL;
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
		if (g_module_symbol(module, "ui_module_init", &symbol)) {

			typedef void (*ui_module_init_t)(ui_config_entry &entry);

			ui_module_init_t ui_module_init = (ui_module_init_t) symbol;

			ui_module_init(*this);
		} else {
			g_warning("failed to call module %s init function\n", ui_lib.c_str());
		}

	} else {
		g_warning("failed to open module %s\n", ui_lib.c_str() );

		//! just to debug
		void *handle = dlopen(ui_lib.c_str(), RTLD_LAZY|RTLD_GLOBAL);
		if (!handle) {
			fprintf(stderr, "dlopen(): %s\n", dlerror());
		} else {
			printf("dlopen %s OK\n", ui_lib.c_str());
			dlclose(handle);
		}
	}

	content = gtk_bin_get_child(GTK_BIN(this->window));
	if (!content) {
		std::cerr << "gtk_bin_get_child() for main window in module "
					<< ui_def << " failed" << std::endl;
	}
	gtk_widget_unparent(content);

	hbox = GTK_HBOX(gtk_hbox_new(FALSE, 5));
	tabicon = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_EXECUTE, GTK_ICON_SIZE_BUTTON));
	tablabel = GTK_LABEL(gtk_label_new(this->program_name.c_str()));
	tabbutton = GTK_BUTTON(gtk_button_new());
	tabcloseicon = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_CLOSE, GTK_ICON_SIZE_MENU));

	gtk_button_set_image(tabbutton, GTK_WIDGET(tabcloseicon));
	gtk_button_set_relief(tabbutton, GTK_RELIEF_NONE);

	g_signal_connect(G_OBJECT (tabbutton), "clicked", G_CALLBACK(tabcloseicon_event_cb), this);

	gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(tabicon));
	gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(tablabel));
	gtk_box_pack_start_defaults(GTK_BOX(hbox), GTK_WIDGET(tabbutton));
	gtk_widget_show_all(GTK_WIDGET(hbox));

	if (gtk_notebook_append_page(getNotebook(), this->content, GTK_WIDGET(this->hbox)) == -1) {
		std::cerr << "gtk_notebook_append_page() for module "
			<< ui_def << " failed" << std::endl;
	} else {
		gtk_notebook_set_tab_reorderable(getNotebook(), this->content, TRUE);
	}

	this->show_page(FALSE);
}

ui_config_entry::~ui_config_entry() {

	if(builder) {
		if(content) {
			int page_no = gtk_notebook_page_num(getNotebook(), this->content);
			if (page_no != -1) {
				gtk_notebook_remove_page(getNotebook(), page_no);
			}
		}
	}

	if (module) {
		gpointer symbol;
		if (g_module_symbol(module, "ui_module_unload", &symbol)) {

			typedef void (*ui_module_unload_t)(void);

			ui_module_unload_t ui_module_unload = (ui_module_unload_t) symbol;

			ui_module_unload();
		}

		if(!g_module_close(module)) {
			const char *module_name = (char *) g_module_name(module);
			fprintf(stderr, "error closing module %s", module_name);
		}
	}
}

ui_config_entry::childrens_t ui_config_entry::getChildByType(ui_config_entry_type _type) {

	childrens_t ret;

	for (childrens_t::iterator Iter = children.begin(); Iter != children.end(); Iter++) {
		if ((*Iter)->type == _type) {
			ret.push_back((*Iter));
		}

		childrens_t childrensOf = (*Iter)->getChildByType(_type);
		for (childrens_t::iterator Iter2 = childrensOf.begin(); Iter2 != childrensOf.end(); Iter2++) {
			ret.push_back((*Iter2));
		}
	}

	return ret;
}

void ui_config_entry::addWidget(ui_widget_entry *entry)
{
	widgetVector.push_back(entry);
}

ui_widget_entry * ui_config_entry::getWidget(gint whichOne)
{
	return widgetVector.at(whichOne);
}

void ui_config_entry::process_spawn(void) {
	if (is_running) {
		throw std::runtime_error(std::string(program_name) + " already running");
	}

	is_running = true;
}

void ui_config_entry::process_kill(int signum) {
	if (!is_running) {
		throw std::runtime_error(std::string(program_name) + " is not running");
	}

	// TODO

	is_running = false;
}

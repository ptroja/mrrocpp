#include <gtk/gtk.h>
#include <glib.h>
#include <gtkmm.h>

#include <iostream>

#include "ui_config_entry.h"
#include "ui_model.h"

using namespace Gtk;

extern "C" {
	//void on_button_clicked(GtkButton *button, gpointer user_data) {
	void on_button_clicked() {
		g_warn_if_reached();
	}
}

class PulseButton : public Gtk::Button {
	public:
		PulseButton(const Glib::ustring & label, const Gtk::StockID& stock_id)
			: ButtonLabel(label), ButtonImage(stock_id, Gtk::ICON_SIZE_BUTTON) {

			ButtonImage.set_alignment(Gtk::ALIGN_RIGHT);
			ButtonBox.pack_start(ButtonImage);
			ButtonBox.pack_start(ButtonLabel);
			this->add(ButtonBox);

			click = this->signal_clicked().connect(sigc::ptr_fun(&on_button_clicked));
		}

		~PulseButton() {
			click.disconnect();
		}

	protected:
		Gtk::HBox ButtonBox;
		Gtk::Label ButtonLabel;
		Gtk::Image ButtonImage;

	private:
		sigc::connection click;
};

class ReaderButtonBox : public Gtk::HButtonBox {
	public:
		ReaderButtonBox()
			: StartButton("Start", Gtk::Stock::MEDIA_PLAY), TriggerButton("Trigger", Gtk::Stock::INDEX) {
			this->add(StartButton);
			this->add(TriggerButton);
		}

	protected:
		PulseButton StartButton, TriggerButton;
};

void mp_module_init(ui_config_entry &entry) {

	GtkBuilder & builder = (entry.getBuilder());

	std::vector <ui_config_entry *> ecps = ui_model::instance().getRootNode().getChildByType(ui_config_entry::ECP);

	GtkTable *pulsetable = GTK_TABLE(gtk_builder_get_object(&builder, "pulsetable"));
	g_assert(pulsetable);

	gint ncolumns, nrows;
	g_object_get(G_OBJECT(pulsetable), "n-columns", &ncolumns, "n-rows", &nrows, NULL);
	printf("nc %d nr %d\n", ncolumns, nrows);

	gtk_table_resize(pulsetable, nrows+ecps.size(), ncolumns);
	g_object_get(G_OBJECT(pulsetable), "n-columns", &ncolumns, "n-rows", &nrows, NULL);
	printf("nc %d nr %d\n", ncolumns, nrows);

	int ecp_num = 0;
	for (std::vector<ui_config_entry *>::iterator Iter = ecps.begin(); Iter != ecps.end(); Iter++, ecp_num++) {

		//! create label widget
		Gtk::Label *ecp_label = new Gtk::Label((*Iter)->program_name.c_str());

		//! attach label widget to table
		gtk_table_attach(pulsetable, GTK_WIDGET(ecp_label->gobj()), // table, widget
				0, 1, // left, right attach
				2+ecp_num, 2+ecp_num+1, // top, bottom attach
				(GtkAttachOptions) 0, (GtkAttachOptions) 0, // x,y options
				0, 0 // x,y padding
				);

		ReaderButtonBox *rbb = new ReaderButtonBox();

		gtk_table_attach(pulsetable, GTK_WIDGET(rbb->gobj()), // table, widget
				2, 3, // left, right attach
				2+ecp_num, 2+ecp_num+1, // top, bottom attach
				(GtkAttachOptions) 0, (GtkAttachOptions) 0, // x,y options
				0, 0 // x,y padding
				);

		gtk_container_child_set(GTK_CONTAINER(pulsetable), GTK_WIDGET(rbb->gobj()), "x-padding", 10, "x-options", GTK_EXPAND|GTK_FILL, NULL);

		PulseButton *ecptriggertbutton = new PulseButton("Trigger", Gtk::Stock::INDEX);

		gtk_table_attach(pulsetable, GTK_WIDGET(ecptriggertbutton->gobj()), // table, widget
				4, 5, // left, right attach
				2+ecp_num, 2+ecp_num+1, // top, bottom attach
				(GtkAttachOptions) 0, (GtkAttachOptions) 0, // x,y options
				0, 0 // x,y padding
		);

		//! show all childer added to the table
		gtk_widget_show_all(GTK_WIDGET(pulsetable));
	}

	//! reposition horizontal separators
	{
		guint bottom, top;

		bottom = 2;
		top = 1;
		gtk_container_child_set(GTK_CONTAINER(pulsetable), GTK_WIDGET(gtk_builder_get_object(&builder, "hseparatorLower")), "bottom-attach", bottom, "top-attach", top, NULL);

		bottom = 2+ecps.size()+1;
		top = 1+ecps.size()+1;
		gtk_container_child_set(GTK_CONTAINER(pulsetable), GTK_WIDGET(gtk_builder_get_object(&builder, "hseparatorUpper")), "bottom-attach", bottom, "top-attach", top, NULL);
	}

	//! reposition boot line
	const char *bottom_line[] = {
			"AllRobotsLabel",
			"AllRobotsReaderPulses",
			"AllRobotsEcpTrigger",
			NULL
	};

	for(const char **widget_name = bottom_line; *widget_name; widget_name++) {
		guint bottom, top;
		GObject *object = gtk_builder_get_object(&builder, *widget_name);

		gtk_container_child_get(GTK_CONTAINER(pulsetable), GTK_WIDGET(object), "bottom-attach", &bottom, "top-attach", &top, NULL);

		bottom += ecps.size();
		top += ecps.size();

		gtk_container_child_set(GTK_CONTAINER(pulsetable), GTK_WIDGET(object), "bottom-attach", bottom, "top-attach", top, NULL);

		printf("widget_name = %s\n", *widget_name);
	}

	const char *vseparators[] = {
			"ReaderSeparator",
			"EcpSeparator",
			NULL
	};

	for(const char **widget_name = vseparators; *widget_name; widget_name++) {
		guint bottom, top;
		GObject *object = gtk_builder_get_object(&builder, *widget_name);

		gtk_container_child_get(GTK_CONTAINER(pulsetable), GTK_WIDGET(object), "bottom-attach", &bottom, "top-attach", &top, NULL);

		bottom += ecps.size();
		top = 0;

		gtk_container_child_set(GTK_CONTAINER(pulsetable), GTK_WIDGET(object), "bottom-attach", bottom, "top-attach", top, NULL);

		printf("widget_name = %s\n", *widget_name);
	}

}

extern "C" {

	void ui_module_init(ui_config_entry &entry) {
		mp_module_init(entry);
	}

}

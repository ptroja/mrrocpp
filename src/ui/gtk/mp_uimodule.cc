#include <gtk/gtk.h>
#include <glib.h>
#include <gtkmm.h>

#include <vector>
#include <iostream>

#include "ui_config_entry.h"
#include "ui_model.h"

using namespace Glib;
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

class MpPanel {
	public:
		MpPanel(ui_config_entry &entry);

		~MpPanel(void);

	private:
		std::vector<Gtk::Widget*> PanelWidgets;
};

MpPanel::~MpPanel(void) {
	for (std::vector<Gtk::Widget *>::iterator Iter = PanelWidgets.begin(); Iter != PanelWidgets.end(); Iter++) {
		delete (*Iter);
	}
	PanelWidgets.clear();
}

MpPanel::MpPanel(ui_config_entry &entry) {

	GtkBuilder & builder = (entry.getBuilder());

	std::vector <ui_config_entry *> ecps = ui_model::instance().getRootNode().getChildByType(ui_config_entry::ECP);

	GtkTable *pulsetable = GTK_TABLE(gtk_builder_get_object(&builder, "pulsetable"));
	g_assert(pulsetable);

	Gtk::Table & PulseTable = *Glib::wrap(pulsetable);

	gint ncolumns, nrows;
	ncolumns = PulseTable.property_n_columns();
	nrows = PulseTable.property_n_rows();

	PulseTable.resize(nrows+ecps.size(), ncolumns);

	int ecp_num = 0;
	for (std::vector<ui_config_entry *>::iterator Iter = ecps.begin(); Iter != ecps.end(); Iter++, ecp_num++) {

		//! create label widget
		Gtk::Label *EcpLabel = new Gtk::Label((*Iter)->program_name.c_str());
		PanelWidgets.push_back(EcpLabel);

		PulseTable.attach(*EcpLabel,
				0, 1, // left, right attach
				2+ecp_num, 2+ecp_num+1, // top, bottom attach
				(AttachOptions) 0, (AttachOptions) 0 // x, y options
		);

		ReaderButtonBox *rbb = new ReaderButtonBox();
		PanelWidgets.push_back(rbb);

		//! attach button box with default expand and padding
		PulseTable.attach(*rbb,
				2, 3, // left, right attach
				2+ecp_num, 2+ecp_num+1); // top, bottom attach

		gtk_container_child_set(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(rbb->gobj()), "x-padding", 10, "x-options", GTK_EXPAND|GTK_FILL, NULL);

		PulseButton *EcpTriggerButton = new PulseButton("Trigger", Gtk::Stock::INDEX);
		PanelWidgets.push_back(EcpTriggerButton);

		PulseTable.attach(*EcpTriggerButton,
				4, 5, // left, right attach
				2+ecp_num, 2+ecp_num+1, // top, bottom attach
				(AttachOptions) 0, (AttachOptions) 0 // x, y options
		);

		//! show all childer added to the table
		PulseTable.show_all();
	}

	//! reposition horizontal separators
	{
		guint bottom, top;

		bottom = 2;
		top = 1;
		gtk_container_child_set(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(gtk_builder_get_object(&builder, "hseparatorLower")), "bottom-attach", bottom, "top-attach", top, NULL);

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

		gtk_container_child_get(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(object), "bottom-attach", &bottom, "top-attach", &top, NULL);

		bottom += ecps.size();
		top += ecps.size();

		gtk_container_child_set(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(object), "bottom-attach", bottom, "top-attach", top, NULL);

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

		gtk_container_child_get(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(object), "bottom-attach", &bottom, "top-attach", &top, NULL);

		bottom += ecps.size();
		top = 0;

		gtk_container_child_set(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(object), "bottom-attach", bottom, "top-attach", top, NULL);

		printf("widget_name = %s\n", *widget_name);
	}

}

static MpPanel *panel;

extern "C" {

	void ui_module_init(ui_config_entry &entry) {
		panel = new MpPanel(entry);
		printf("module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) {
		if (panel) {
			delete panel;
		}
		printf("module %s unloaded\n", __FILE__);
	}
}

#include <gtk/gtk.h>
#include <glib.h>
#include <gtkmm.h>

#include <boost/foreach.hpp>

#include <vector>
#include <iostream>

#include <sys/types.h>
#include <sys/wait.h>
#include <csignal>

#include "ui_config_entry.h"
#include "ui_model.h"

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#endif

using namespace Glib;
using namespace Gtk;

extern "C" {
	//void on_button_clicked(GtkButton *button, gpointer user_data);
	void on_button_clicked();
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

		void execute_mp_pulse(int pulse_code, int pulse_value = 1);
		void StartButtonSensitive(bool sensitive);
		void StopButtonSensitive(bool sensitive);
		void TriggerButtonSensitive(bool sensitive);

		enum StartButtonMode_t{ START, PAUSE, RESUME};
		void StartButtonMode(StartButtonMode_t mode);

		typedef enum { STARTED, PAUSED, STOPPED, UNLOADED} MP_state_t;

		MP_state_t getMP_state() const
		{
			return MP_state;
		}

		void setMP_state(MP_state_t state)
		{
			this->MP_state = state;
		}

	private:
		ui_config_entry & config_entry;
		std::vector<Gtk::Widget*> PanelWidgets;
		pid_t mp_pid;
		MP_state_t MP_state;
		messip_channel_t *pulse_fd;
		GtkButton *MpStartPauseButton, *MpStopButton, *MpTriggerButton,
					*AllRobotsReaderButton, *AllRobotsReaderTriggerButton, *AllRobotsEcpTrigger;
};

MpPanel::MpPanel(ui_config_entry &entry)
	: config_entry(entry), MP_state(UNLOADED)
	{

	GtkBuilder & builder = (entry.getBuilder());

	// Assign objects
	struct builder_widget {
		GtkButton **object;
		const char *name;
	} builder_widgets[] = {
			{ &MpStartPauseButton, "mpStartPauseButton" },
			{ &MpStopButton, "mpStopButton" },
			{ &MpTriggerButton, "mpTriggerButton" },
			{ &AllRobotsReaderButton, "AllRobotsReaderButton" },
			{ &AllRobotsReaderTriggerButton, "AllRobotsReaderTriggerButton" },
			{ &AllRobotsEcpTrigger, "AllRobotsEcpTrigger" },
	};

	for (size_t i = 0; i < sizeof(builder_widgets)/sizeof(builder_widgets[0]); i++) {
		*builder_widgets[i].object = GTK_BUTTON(gtk_builder_get_object(&builder, builder_widgets[i].name));
		if(!*builder_widgets[i].object) {
			fprintf(stderr, "mp button object %d (\"%s\") assignment failed\n", i, builder_widgets[i].name);

		}
		g_assert(*builder_widgets[i].object);
	}

	// get the ECPs
	ui_config_entry::childrens_t ecps(ui_model::instance().getRootNode().getChildByType(ui_config_entry::ECP));

	GtkTable *pulsetable = GTK_TABLE(gtk_builder_get_object(&builder, "pulsetable"));
	g_assert(pulsetable);

	Gtk::Table & PulseTable = *Glib::wrap(pulsetable);

	gint ncolumns, nrows;
	ncolumns = PulseTable.property_n_columns();
	nrows = PulseTable.property_n_rows();

	PulseTable.resize(nrows+ecps.size(), ncolumns);

	int ecp_num = 0;
	for (ui_config_entry::childrens_t::iterator Iter = ecps.begin(); Iter != ecps.end(); Iter++, ecp_num++) {

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

		// there should be some C++ method insted C call
		// http://bugzilla.gnome.org/show_bug.cgi?id=140515
		gtk_container_child_set(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(rbb->gobj()),
				"x-padding", 10, "x-options", GTK_EXPAND|GTK_FILL, (void *) NULL);

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
		gtk_container_child_set(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(gtk_builder_get_object(&builder, "hseparatorLower")), "bottom-attach", bottom, "top-attach", top, (void *) NULL);

		bottom = 2+ecps.size()+1;
		top = 1+ecps.size()+1;
		gtk_container_child_set(GTK_CONTAINER(pulsetable), GTK_WIDGET(gtk_builder_get_object(&builder, "hseparatorUpper")), "bottom-attach", bottom, "top-attach", top, (void *) NULL);
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

		gtk_container_child_get(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(object), "bottom-attach", &bottom, "top-attach", &top, (void *) NULL);

		bottom += ecps.size();
		top += ecps.size();

		gtk_container_child_set(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(object), "bottom-attach", bottom, "top-attach", top, (void *) NULL);
	}

	const char *vseparators[] = {
			"ReaderSeparator",
			"ecpSeparator",
			NULL
	};

	for(const char **widget_name = vseparators; *widget_name; widget_name++) {
		guint bottom, top;
		GObject *object = gtk_builder_get_object(&builder, *widget_name);

		gtk_container_child_get(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(object), "bottom-attach", &bottom, "top-attach", &top, (void *) NULL);

		bottom += ecps.size();
		top = 0;

		gtk_container_child_set(GTK_CONTAINER(PulseTable.gobj()), GTK_WIDGET(object), "bottom-attach", bottom, "top-attach", top, (void *) NULL);
	}

	//! spawn MP
	mp_pid = ui_model::instance().getConfigurator().process_spawn(lib::MP_SECTION);

	if (mp_pid > 0) {

		const std::string network_pulse_attach_point = ui_model::instance().getConfigurator()
			.return_attach_point_name(lib::configurator::CONFIG_SERVER, "mp_pulse_attach_point", lib::MP_SECTION);

		short tmp = 0;
		// try to open channel
		while( (pulse_fd = messip::port_connect(network_pulse_attach_point)) == NULL)
			if((tmp++)<lib::CONNECT_RETRY) {
//				fprintf(stderr, "."); fflush(stderr);
				delay(lib::CONNECT_DELAY);
			} else {
				fprintf(stderr, "blad odwolania do: %s,\n", network_pulse_attach_point.c_str());
				break;
			}
	} else {
		fprintf(stderr, "mp spawn failed\n");
		return;
	}

	StartButtonSensitive(true);
	MP_state = STOPPED;
}

MpPanel::~MpPanel(void) {

	BOOST_FOREACH(Gtk::Widget * widget, PanelWidgets) {
		delete widget;
	}

/*
	if ((interface.mp.state == UI_MP_TASK_RUNNING) || (interface.mp.state == UI_MP_TASK_PAUSED)){

		pulse_stop_mp (widget,apinfo,cbinfo);
	}
*/

	if (pulse_fd) {
		messip::port_disconnect(pulse_fd);
	}

	if (mp_pid > 0) {
		if(kill(mp_pid, SIGTERM) == -1) {
			perror("kill()");
			return;
		}
		if(waitpid(mp_pid, NULL, 0) == -1) {
			perror("waitpid()");
		}
	}
}

void MpPanel::execute_mp_pulse (int pulse_code, int pulse_value)
{
	if (pulse_fd) {
		// send 1WAY empty message
		if (messip::port_send_pulse(pulse_fd, pulse_code, pulse_value) ==-1) {
			  perror("messip::port_send_pulse");
			  throw;
		}
	}
}

void MpPanel::StartButtonSensitive(bool sensitive) {
	gtk_widget_set_sensitive(GTK_WIDGET(MpStartPauseButton), sensitive);
}

void MpPanel::StopButtonSensitive(bool sensitive) {
	gtk_widget_set_sensitive(GTK_WIDGET(MpStopButton), sensitive);
}

void MpPanel::TriggerButtonSensitive(bool sensitive) {
	gtk_widget_set_sensitive(GTK_WIDGET(MpTriggerButton), sensitive);
}

void MpPanel::StartButtonMode(StartButtonMode_t mode) {
	// get the builder
	GtkBuilder & builder = (config_entry.getBuilder());

	// get the button image
	GtkImage *button_image = GTK_IMAGE(gtk_builder_get_object(&builder, "mpStartButtonImage"));
	g_assert(button_image);

	// get the stock image size
	char *stock_id;
	GtkIconSize icon_size;
	gtk_image_get_stock(button_image, &stock_id, &icon_size);

	// get the button label
	GtkLabel *button_label = GTK_LABEL(gtk_builder_get_object(&builder, "mpStartButtonLabel"));
	g_assert(button_label);

	switch (mode) {
		case START:
			gtk_image_set_from_stock(button_image, GTK_STOCK_MEDIA_PLAY, icon_size);
			gtk_label_set_label(button_label, "Start");
			break;
		case PAUSE:
			gtk_image_set_from_stock(button_image, GTK_STOCK_MEDIA_PAUSE, icon_size);
			gtk_label_set_label(button_label, "Pause");
			break;
		case RESUME:
			gtk_image_set_from_stock(button_image, GTK_STOCK_MEDIA_PLAY, icon_size);
			gtk_label_set_label(button_label, "Resume");
			break;
	}
}


extern "C" {
	//void on_button_clicked(GtkButton *button, gpointer user_data) {
	void on_button_clicked() {
		g_warn_if_reached();
	}

	void on_MpStartPauseButton_clicked(GtkButton *button, gpointer user_data) {
		// get the mp object
		ui_config_entry & entry = *(ui_config_entry *) user_data;
		MpPanel & mp = *(MpPanel *) entry.user_data;

		switch(mp.getMP_state()) {
			case MpPanel::STOPPED:
				// execute pulse
				mp.execute_mp_pulse(MP_START);

				// manage the buttons
				mp.StartButtonMode(MpPanel::PAUSE);
				mp.StopButtonSensitive(true);
				mp.TriggerButtonSensitive(true);

				mp.setMP_state(MpPanel::STARTED);
				break;
			case MpPanel::STARTED:
				// execute pulse
				mp.execute_mp_pulse(MP_PAUSE);

				// manage the buttons
				mp.StartButtonMode(MpPanel::RESUME);
				mp.StopButtonSensitive(true);
				mp.TriggerButtonSensitive(true);

				mp.setMP_state(MpPanel::PAUSED);
				break;
			case MpPanel::PAUSED:
				// execute pulse
				mp.execute_mp_pulse(MP_RESUME);

				// manage the buttons
				mp.StartButtonMode(MpPanel::PAUSE);
				mp.StopButtonSensitive(true);
				mp.TriggerButtonSensitive(true);

				mp.setMP_state(MpPanel::STARTED);
				break;
			default:
				break;
		}

	}

	void on_MpStopButton_clicked(GtkButton *button, gpointer user_data) {
		// get the mp object
		ui_config_entry & entry = *(ui_config_entry *) user_data;
		MpPanel & mp = *(MpPanel *) entry.user_data;

		// execute pulse
		mp.execute_mp_pulse(MP_STOP);

		// manage the buttons
		mp.StartButtonMode(MpPanel::START);
		mp.StopButtonSensitive(false);
		mp.TriggerButtonSensitive(false);
		mp.setMP_state(MpPanel::STOPPED);
	}

	void on_MpTriggerButton_clicked(GtkButton *button, gpointer user_data) {
		// get the mp object
		ui_config_entry & entry = *(ui_config_entry *) user_data;
		MpPanel & mp = *(MpPanel *) entry.user_data;

		// execute pulse
		mp.execute_mp_pulse(MP_TRIGGER);
	}
}

static MpPanel *panel;

extern "C" {

	void ui_module_init(ui_config_entry &entry) {
		panel = new MpPanel(entry);
		entry.user_data = panel;
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) {
		if (panel) {
			delete panel;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

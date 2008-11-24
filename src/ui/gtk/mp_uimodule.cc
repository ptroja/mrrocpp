#include <gtk/gtk.h>
#include <glib.h>

#include <iostream>

#include "ui_config_entry.h"
#include "ui_model.h"

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

		printf("ECP++ %s @ (%d,%d)\n", (*Iter)->program_name.c_str(), 2+ecp_num, 2+ecp_num+1);

		//! create label widget
		GtkWidget *ecp_label = gtk_label_new((*Iter)->program_name.c_str());
		gtk_widget_show(ecp_label);

		//! attach label widget to table
		gtk_table_attach(pulsetable, ecp_label, // table, widget
				0, 1, // left, right attach
				2+ecp_num, 2+ecp_num+1, // top, bottom attach
				(GtkAttachOptions) 0, (GtkAttachOptions) 0, // x,y options
				0, 0 // x,y padding
				);

		GtkHButtonBox *readerbuttonbox = GTK_HBUTTON_BOX(gtk_hbutton_box_new());
			GtkButton *startbutton = GTK_BUTTON(gtk_button_new());
				GtkHBox *startbuttonbox = GTK_HBOX(gtk_hbox_new(TRUE, 0));
					GtkImage *startimage = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_MEDIA_PLAY, GTK_ICON_SIZE_BUTTON));
					GtkLabel *startlabel = GTK_LABEL(gtk_label_new("Start"));
			GtkButton *triggerbutton = GTK_BUTTON(gtk_button_new());
				GtkHBox *triggerbuttonbox = GTK_HBOX(gtk_hbox_new(TRUE, 0));
					GtkImage *triggerimage = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_INDEX, GTK_ICON_SIZE_BUTTON));
					GtkLabel *triggerlabel = GTK_LABEL(gtk_label_new("Trigger"));

		g_object_set(G_OBJECT(triggerimage), "xalign", 1.00, NULL);
		gtk_box_pack_start_defaults(GTK_BOX(triggerbuttonbox), GTK_WIDGET(triggerimage));
		gtk_box_pack_start_defaults(GTK_BOX(triggerbuttonbox), GTK_WIDGET(triggerlabel));
		//gtk_container_child_set(GTK_CONTAINER(triggerbuttonbox), GTK_WIDGET(triggerimage), "fill", TRUE, NULL);
		gtk_container_add(GTK_CONTAINER(triggerbutton), GTK_WIDGET(triggerbuttonbox));

		g_object_set(G_OBJECT(startimage), "xalign", 1.00, NULL);
		gtk_box_pack_start_defaults(GTK_BOX(startbuttonbox), GTK_WIDGET(startimage));
		gtk_box_pack_start_defaults(GTK_BOX(startbuttonbox), GTK_WIDGET(startlabel));
		gtk_container_add(GTK_CONTAINER(startbutton), GTK_WIDGET(startbuttonbox));

		gtk_container_add(GTK_CONTAINER(readerbuttonbox), GTK_WIDGET(startbutton));
		gtk_container_add(GTK_CONTAINER(readerbuttonbox), GTK_WIDGET(triggerbutton));

		gtk_widget_show_all(GTK_WIDGET(readerbuttonbox));

		gtk_table_attach(pulsetable, GTK_WIDGET(readerbuttonbox), // table, widget
				2, 3, // left, right attach
				2+ecp_num, 2+ecp_num+1, // top, bottom attach
				(GtkAttachOptions) 0, (GtkAttachOptions) 0, // x,y options
				0, 0 // x,y padding
				);

		gtk_container_child_set(GTK_CONTAINER(pulsetable), GTK_WIDGET(readerbuttonbox), "x-padding", 10, "x-options", GTK_EXPAND|GTK_FILL, NULL);

		GtkButton *ecptriggertbutton = GTK_BUTTON(gtk_button_new());
			GtkHBox *ecptriggerbuttonbox = GTK_HBOX(gtk_hbox_new(TRUE, 0));
				GtkImage *ecptriggerimage = GTK_IMAGE(gtk_image_new_from_stock(GTK_STOCK_INDEX, GTK_ICON_SIZE_BUTTON));
				GtkLabel *ecptriggerlabel = GTK_LABEL(gtk_label_new("Trigger"));

		g_object_set(G_OBJECT(ecptriggerimage), "xalign", 1.00, NULL);
		gtk_box_pack_start_defaults(GTK_BOX(ecptriggerbuttonbox), GTK_WIDGET(ecptriggerimage));
		gtk_box_pack_start_defaults(GTK_BOX(ecptriggerbuttonbox), GTK_WIDGET(ecptriggerlabel));
		gtk_container_add(GTK_CONTAINER(ecptriggertbutton), GTK_WIDGET(ecptriggerbuttonbox));

		gtk_widget_show_all(GTK_WIDGET(ecptriggertbutton));

		gtk_table_attach(pulsetable, GTK_WIDGET(ecptriggertbutton), // table, widget
				4, 5, // left, right attach
				2+ecp_num, 2+ecp_num+1, // top, bottom attach
				(GtkAttachOptions) 0, (GtkAttachOptions) 0, // x,y options
				0, 0 // x,y padding
		);
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

	void on_button1_activate(GtkObject *object, gpointer user_data)
	{
		g_warn_if_reached();
	}


	void on_button1_clicked(GtkObject *object, gpointer user_data)
	{
		g_warn_if_reached();
	}

	void ui_module_init(ui_config_entry &entry) {
		mp_module_init(entry);
	}

}

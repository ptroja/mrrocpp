
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6m_servo_algorithm_widget.h"


edp_irp6m_servo_algorithm::edp_irp6m_servo_algorithm(ui_widget_entry &entry) 
{
}

static edp_irp6m_servo_algorithm *servo_mechatronika;


extern "C"
{
	void on_arrow_button_clicked_mechatronika_servo (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entry1)));
	
        GtkEntry * entry2 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, atof(gtk_entry_get_text(entry2)));
	
        GtkEntry * entry3 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, atof(gtk_entry_get_text(entry3)));
	
        GtkEntry * entry4 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4"));
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, atof(gtk_entry_get_text(entry4)));
	
        GtkEntry * entry5 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, atof(gtk_entry_get_text(entry5)));
	
        GtkEntry * entry6 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, atof(gtk_entry_get_text(entry6)));
	
        GtkEntry * entry7 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, atof(gtk_entry_get_text(entry7)));
	
        GtkEntry * entry8 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry8"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, atof(gtk_entry_get_text(entry8)));
	
        GtkEntry * entry9 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry9"));
        GtkSpinButton * spin9 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton9"));
        gtk_spin_button_set_value(spin9, atof(gtk_entry_get_text(entry9)));
	
        GtkEntry * entry10 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry10"));
        GtkSpinButton * spin10 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton10"));
        gtk_spin_button_set_value(spin10, atof(gtk_entry_get_text(entry10)));
	
	}
	
	void on_read_button_clicked_mechatronika_servo (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla mechatronika servo" << std::endl;
	}
	
	void on_set_button_clicked_mechatronika_servo (GtkButton* button, gpointer user_data)
	{
		std::cout << "ustaw wartosci dla mechatronika servo" << std::endl;
	}
	
	
	void ui_widget_init(ui_widget_entry &entry) 
	{
		servo_mechatronika = new edp_irp6m_servo_algorithm(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (servo_mechatronika) 
		{
			delete servo_mechatronika;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
}

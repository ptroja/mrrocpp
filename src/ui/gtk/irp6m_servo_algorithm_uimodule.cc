#include <gtk/gtk.h>
#include <glib.h>


#include <iostream>

#include "ui_model.h"
#include "irp6m_servo_algorithm.h"


edp_irp6m_servo_algorithm::edp_irp6m_servo_algorithm(ui_config_entry &entry) 
{
}

static edp_irp6m_servo_algorithm *servo;


extern "C"
{
	void on_arrow_button_clicked_m (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla mechatroniki" << std::endl;
		
		ui_config_entry & servoEntry = *(ui_config_entry *) userdata;
		GtkBuilder & copyBuilder = (servoEntry.getBuilder());
		
		GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&copyBuilder, "entry1"));
		GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&copyBuilder, "spinbutton1"));
		gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entry1)));
		
		GtkEntry * entry2 = GTK_ENTRY(gtk_builder_get_object(&copyBuilder, "entry2"));
		GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&copyBuilder, "spinbutton2"));
		gtk_spin_button_set_value(spin2, atof(gtk_entry_get_text(entry2)));
		
		//reszta bedzie dodana za pomoca generatora :)
		
		
	}
	
	void on_read_button_clicked_m (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla mechatroniki" << std::endl;
	}
	
	void on_set_button_clicked_m (GtkButton* button, gpointer user_data)
	{
		std::cout << "ustaw wartosci dla mechatroniki" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		servo = new edp_irp6m_servo_algorithm(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (servo) 
		{
			delete servo;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

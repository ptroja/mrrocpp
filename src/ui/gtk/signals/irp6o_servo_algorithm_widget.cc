
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6o_servo_algorithm_widget.h"


edp_irp6o_servo_algorithm::edp_irp6o_servo_algorithm(ui_widget_entry &entry) 
{
}

static edp_irp6o_servo_algorithm *servo_ontrack;


extern "C"
{
	void on_arrow_button_clicked_ontrack_servo (GtkButton* button, gpointer userdata)
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
	
        GtkEntry * entry11 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry11"));
        GtkSpinButton * spin11 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton11"));
        gtk_spin_button_set_value(spin11, atof(gtk_entry_get_text(entry11)));
	
        GtkEntry * entry12 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry12"));
        GtkSpinButton * spin12 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton12"));
        gtk_spin_button_set_value(spin12, atof(gtk_entry_get_text(entry12)));
	
        GtkEntry * entry13 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry13"));
        GtkSpinButton * spin13 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton13"));
        gtk_spin_button_set_value(spin13, atof(gtk_entry_get_text(entry13)));
	
        GtkEntry * entry14 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry14"));
        GtkSpinButton * spin14 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton14"));
        gtk_spin_button_set_value(spin14, atof(gtk_entry_get_text(entry14)));
	
        GtkEntry * entry15 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry15"));
        GtkSpinButton * spin15 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton15"));
        gtk_spin_button_set_value(spin15, atof(gtk_entry_get_text(entry15)));
	
        GtkEntry * entry16 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry16"));
        GtkSpinButton * spin16 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton16"));
        gtk_spin_button_set_value(spin16, atof(gtk_entry_get_text(entry16)));
	
	}
	
	void on_read_button_clicked_ontrack_servo (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
        GtkEntry * entry2 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2"));

		BYTE servo_alg_no[8];
		BYTE servo_par_no[8];
		
			if (robot->ecp->get_EDP_pid()!=-1)
			{
				if (state.is_synchronised)  // Czy robot jest zsynchronizowany?
				{
					if (!(robot->get_servo_algorithm(servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
						printf("Blad w mechatronika get_servo_algorithm\n");
					
						gtk_entry_set_text(entry1, (const gchar*)servo_alg_no[0]);
						gtk_entry_set_text(entry2, (const gchar*)servo_par_no[0]);						
				} else
				{
					std::cout << "I am not synchronized yet!!!" << std::endl;
				}
			}
	}
	
	void on_set_button_clicked_ontrack_servo (GtkButton* button, gpointer userdata)
	{

	gint servo_alg_no_tmp [8];
	BYTE servo_alg_no_output[8];
	gint servo_par_no_tmp [8];
	BYTE servo_par_no_output[8];
	
	ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
    GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
    
    GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
    GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));

	if (state.is_synchronised)
	{
		servo_alg_no_tmp[0] = gtk_spin_button_get_value_as_int(spin1);
		servo_par_no_tmp[0] = gtk_spin_button_get_value_as_int(spin2);
		
		std::cout << servo_alg_no_tmp[0] << std::endl;
		std::cout << servo_par_no_tmp[0] << std::endl;

		servo_alg_no_output[0] = (BYTE)(servo_alg_no_tmp[0]);
		servo_par_no_output[0] = (BYTE)(servo_par_no_tmp[0]);
		
		std::cout << servo_alg_no_output[0] << std::endl;
		std::cout << servo_par_no_output[0] << std::endl;
		
		//for(int i=0; i<IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++)
		//for(int i=0; i<2; i++)
		//{
		//	servo_alg_no_output[i] = BYTE(servo_alg_no_tmp[i]);
		//	servo_par_no_output[i] = BYTE(servo_par_no_tmp[i]);
		//}

		// zlecenie wykonania ruchu
		//robot->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

	}
	else
	{
	}

	}
	
	
	void ui_widget_init(ui_widget_entry &entry) 
	{
		servo_ontrack = new edp_irp6o_servo_algorithm(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (servo_ontrack) 
		{
			delete servo_ontrack;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
}

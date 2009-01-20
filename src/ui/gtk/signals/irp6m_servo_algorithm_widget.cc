
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6m_servo_algorithm_widget.h"


#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "ui/ui_ecp.h"

ui_msg_def ui_msg;
ui_state_def ui_state;
ui_robot_def ui_robot;


double irp6m_current_pos[6]; // pozycja biezaca
double irp6m_desired_pos[6]; // pozycja zadana

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
	
	void on_read_button_clicked_mechatronika_servo (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
        GtkEntry * entry2 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2"));
        GtkEntry * entry3 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3"));
        GtkEntry * entry4 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4"));
        GtkEntry * entry5 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5"));
        GtkEntry * entry6 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6"));
        GtkEntry * entry7 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7"));
        GtkEntry * entry8 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry8"));
        GtkEntry * entry9 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry9"));
        GtkEntry * entry10 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry10"));
        
		BYTE servo_alg_no[IRP6_MECHATRONIKA_NUM_OF_SERVOS];
		BYTE servo_par_no[IRP6_MECHATRONIKA_NUM_OF_SERVOS];
		
		gchar * servo_alg_no_gchar = (gchar*)(servo_alg_no);
		gchar * servo_par_no_gchar = (gchar*)(servo_par_no);		

		try
			{
			if (ui_state.irp6_mechatronika.edp.pid!=-1)
			{
				if ( ui_state.irp6_mechatronika.edp.is_synchronised )  // Czy robot jest zsynchronizowany?
				{
					if (!(ui_robot.irp6_mechatronika->get_servo_algorithm(servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
						printf("Blad w mechatronika get_servo_algorithm\n");
					
						gtk_entry_set_text(entry1, &servo_alg_no_gchar[0]);
						gtk_entry_set_text(entry2, &servo_par_no_gchar[0]);
						gtk_entry_set_text(entry3, &servo_alg_no_gchar[1]);
						gtk_entry_set_text(entry4, &servo_par_no_gchar[1]);
						gtk_entry_set_text(entry5, &servo_alg_no_gchar[2]);
						gtk_entry_set_text(entry6, &servo_par_no_gchar[2]);
						gtk_entry_set_text(entry7, &servo_alg_no_gchar[3]);
						gtk_entry_set_text(entry8, &servo_par_no_gchar[3]);
						gtk_entry_set_text(entry9, &servo_alg_no_gchar[4]);
						gtk_entry_set_text(entry10, &servo_par_no_gchar[4]);
						
				} 
				else
				{
					std::cout << "testuje - ale mechatronika nie jest zsynchronizowany" << std::endl;
				}
			}
			else
			{
				std::cout << "testuje - ale pid = 1" << std::endl;;
			}
		

			} // end try
			CATCH_SECTION_UI
	
			
	}
	
	void on_set_button_clicked_mechatronika_servo (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        GtkSpinButton * spin9 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton9"));
        GtkSpinButton * spin10 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton10"));
 
        double servo_alg_no_tmp [IRP6_MECHATRONIKA_NUM_OF_SERVOS];
		BYTE servo_alg_no_output[IRP6_MECHATRONIKA_NUM_OF_SERVOS];
		double servo_par_no_tmp [IRP6_MECHATRONIKA_NUM_OF_SERVOS];
		BYTE servo_par_no_output[IRP6_MECHATRONIKA_NUM_OF_SERVOS];
				
		// wychwytania ew. bledow ECP::robot
		try
		{
		if ( ui_state.irp6_mechatronika.edp.is_synchronised )
		{
			servo_alg_no_tmp[0] = gtk_spin_button_get_value(spin1);
			servo_par_no_tmp[0] = gtk_spin_button_get_value(spin2);
			servo_alg_no_tmp[1] = gtk_spin_button_get_value(spin3);
			servo_par_no_tmp[1] = gtk_spin_button_get_value(spin4);
			servo_alg_no_tmp[2] = gtk_spin_button_get_value(spin5);
			servo_par_no_tmp[2] = gtk_spin_button_get_value(spin6);
			servo_alg_no_tmp[3] = gtk_spin_button_get_value(spin7);
			servo_par_no_tmp[3] = gtk_spin_button_get_value(spin8);
			servo_alg_no_tmp[4] = gtk_spin_button_get_value(spin9);
			servo_par_no_tmp[4] = gtk_spin_button_get_value(spin10);
				
			for(int i=0; i<IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++)
			{
				servo_alg_no_output[i] = servo_alg_no_tmp[i];
				servo_par_no_output[i] = servo_par_no_tmp[i];
			}
		
			// zlecenie wykonania ruchu
			ui_robot.irp6_mechatronika->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);
			
		}
		else
		{
			std::cout << "testuje - ale mechatronika nie jest zsynchronizowany" << std::endl;
		}
		} // end try
		CATCH_SECTION_UI

		return;

	

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

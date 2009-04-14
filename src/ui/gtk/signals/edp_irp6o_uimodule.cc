
#include <iostream>

#include <gtk/gtk.h>
#include <glib.h>

#include "ui_model.h"
#include "edp_irp6o_uimodule.h"


lib::BYTE servo_alg_no[8];
lib::BYTE servo_par_no[8];
	
gint servo_alg_no_tmp [8];
lib::BYTE servo_alg_no_output[8];
gint servo_par_no_tmp [8];
lib::BYTE servo_par_no_output[8];

char buf[32];
gchar buffer[500];
double tool_vector_a[7];
double tool_vector_e[6];
double alfa, kx, ky, kz;
double wl; 
double l_eps = 0;
double irp6o_current_pos_a[9]; // pozycja biezaca
double irp6o_desired_pos_a[9]; // pozycja zadana
double irp6o_current_pos_e[8]; // pozycja biezaca
double irp6o_desired_pos_e[8]; // pozycja zadana
double irp6o_current_pos[8]; // pozycja biezaca
double irp6o_desired_pos[8]; // pozycja zadana



#include "ui/ui_ecp_r_irp6_common.h"

edp_irp6o::edp_irp6o(ui_config_entry &entry)
{
				robot = new ui_common_robot(
				ui_model::instance().getConfigurator(),
				&ui_model::instance().getEcpSr()
				,ROBOT_IRP6_ON_TRACK
				);
				
				robot->get_controller_state(&state);
}

edp_irp6o::~edp_irp6o()
{
	if (robot) {
		delete robot;
	}		
}

static edp_irp6o *edp_ontrack;


extern "C" 
{ 
	void  on_combobox1_changed_ontrack(GtkComboBox *comboBox, gpointer userdata)  
	{
		ui_config_entry & ChoseEntry = *(ui_config_entry *) userdata;
		GtkBuilder & builder = (ChoseEntry.getBuilder());
		
		GtkScrolledWindow * scrolled = GTK_SCROLLED_WINDOW (gtk_builder_get_object(&builder, "scrolledwindow_edp"));

		//if the child exists, destroy it
		if (gtk_bin_get_child(GTK_BIN(scrolled))!=NULL)
		{
			GtkWidget* child = gtk_bin_get_child(GTK_BIN(scrolled));
			gtk_widget_destroy(child);
		}

		gboolean isFile = 0;
		const gchar * windowName;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox);

		switch (choice)
		{
		case 0: std::cout << "Servo algorithm window chosen" << std::endl; isFile = 1; windowName = "window_servo"; break;
		case 1: std::cout << "Internal window chosen" << std::endl; isFile = 1; windowName = "window_int"; break;
		case 2: std::cout << "Increment window chosen" << std::endl; isFile = 1; windowName = "window_inc"; break;
		case 3: std::cout << "XYZ Angle Axis window chosen" << std::endl; isFile = 1; windowName = "window_axis_xyz"; break;
		case 4: std::cout << "XYZ Euler ZYZ window chosen" << std::endl; isFile = 1; windowName = "window_euler_xyz"; break;
		case 5: std::cout << "TS Angle Axis window chosen" << std::endl; isFile = 1; windowName = "window_axis_ts"; break;
		case 6: std::cout << "TS Euler ZYZ window chosen" << std::endl; isFile = 1; windowName = "window_euler_ts"; break;
		default: std::cout << "Something is not working properly!" << std::endl;
		}
		
		if (isFile)
		{
	
			GtkWidget* chosenWindow = GTK_WIDGET (gtk_builder_get_object (&builder, windowName));
			g_assert(chosenWindow);
			
			GtkWidget* windowWithoutParent = gtk_bin_get_child(GTK_BIN(chosenWindow));
			gtk_widget_unparent(windowWithoutParent);
			
			gtk_scrolled_window_add_with_viewport (scrolled, windowWithoutParent);
		}
		
	}	

	void ui_module_init(ui_config_entry &entry) 
	{
		edp_ontrack = new edp_irp6o(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (edp_ontrack) 
		{
			delete edp_ontrack;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}


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
		GtkEntry * entry3 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3"));
		GtkEntry * entry4 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4"));
		GtkEntry * entry5 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5"));
		GtkEntry * entry6 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6"));
		GtkEntry * entry7 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7"));
		GtkEntry * entry8 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry8"));
		GtkEntry * entry9 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry9"));
		GtkEntry * entry10 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry10"));
		GtkEntry * entry11 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry11"));
		GtkEntry * entry12 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry12"));
		GtkEntry * entry13 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry13"));
		GtkEntry * entry14 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry14"));
		GtkEntry * entry15 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry15"));
		GtkEntry * entry16 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry16"));


		if (robot->ecp->get_EDP_pid()!=-1)
		{
				if (state.is_synchronised)  // Czy robot jest zsynchronizowany?
				{
					if (!(robot->get_servo_algorithm(servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
						printf("Blad w mechatronika get_servo_algorithm\n");
					
					gtk_entry_set_text(entry1, (const gchar*)servo_alg_no[0]);
					gtk_entry_set_text(entry2, (const gchar*)servo_par_no[0]);	
					gtk_entry_set_text(entry3, (const gchar*)servo_alg_no[1]);
					gtk_entry_set_text(entry4, (const gchar*)servo_par_no[1]);	
					gtk_entry_set_text(entry5, (const gchar*)servo_alg_no[2]);
					gtk_entry_set_text(entry6, (const gchar*)servo_par_no[2]);	
					gtk_entry_set_text(entry7, (const gchar*)servo_alg_no[3]);
					gtk_entry_set_text(entry8, (const gchar*)servo_par_no[3]);	
					gtk_entry_set_text(entry9, (const gchar*)servo_alg_no[4]);
					gtk_entry_set_text(entry10, (const gchar*)servo_par_no[4]);	
					gtk_entry_set_text(entry11, (const gchar*)servo_alg_no[5]);
					gtk_entry_set_text(entry12, (const gchar*)servo_par_no[5]);	
					gtk_entry_set_text(entry13, (const gchar*)servo_alg_no[6]);
					gtk_entry_set_text(entry14, (const gchar*)servo_par_no[6]);	
					gtk_entry_set_text(entry15, (const gchar*)servo_alg_no[7]);
					gtk_entry_set_text(entry16, (const gchar*)servo_par_no[7]);	
					
				} else
				{
					std::cout << "I am not synchronized yet!!!" << std::endl;
				}
			}
	}
	
	void on_set_button_clicked_ontrack_servo (GtkButton* button, gpointer userdata)
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
		GtkSpinButton * spin11 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton11"));
		GtkSpinButton * spin12 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton12"));
		GtkSpinButton * spin13 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton13"));
		GtkSpinButton * spin14 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton14"));
		GtkSpinButton * spin15 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton15"));
		GtkSpinButton * spin16 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton16"));

 		
 		if (state.is_synchronised)
		{
			servo_alg_no_tmp[0] = gtk_spin_button_get_value_as_int(spin1);
			servo_par_no_tmp[0] = gtk_spin_button_get_value_as_int(spin2);
			servo_alg_no_tmp[1] = gtk_spin_button_get_value_as_int(spin3);
			servo_par_no_tmp[1] = gtk_spin_button_get_value_as_int(spin4);
			servo_alg_no_tmp[2] = gtk_spin_button_get_value_as_int(spin5);
			servo_par_no_tmp[2] = gtk_spin_button_get_value_as_int(spin6);
			servo_alg_no_tmp[3] = gtk_spin_button_get_value_as_int(spin7);
			servo_par_no_tmp[3] = gtk_spin_button_get_value_as_int(spin8);
			servo_alg_no_tmp[4] = gtk_spin_button_get_value_as_int(spin9);
			servo_par_no_tmp[4] = gtk_spin_button_get_value_as_int(spin10);
			servo_alg_no_tmp[5] = gtk_spin_button_get_value_as_int(spin11);
			servo_par_no_tmp[5] = gtk_spin_button_get_value_as_int(spin12);
			servo_alg_no_tmp[6] = gtk_spin_button_get_value_as_int(spin13);
			servo_par_no_tmp[6] = gtk_spin_button_get_value_as_int(spin14);
			servo_alg_no_tmp[7] = gtk_spin_button_get_value_as_int(spin15);
			servo_par_no_tmp[7] = gtk_spin_button_get_value_as_int(spin16);


		for(int i=0; i<8; i++)
		{
			servo_alg_no_output[i] = lib::BYTE(servo_alg_no_tmp[i]);
			servo_par_no_output[i] = lib::BYTE(servo_par_no_tmp[i]);
		}

		// zlecenie wykonania ruchu
		robot->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

	}
	else
	{
		std::cout << "I am not synchronized yet!!!" << std::endl;
	}
 		
	}
}


extern "C"
{
	void on_arrow_button_clicked_ontrack_int (GtkButton* button, gpointer userdata)
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
	
	}
	
	void on_read_button_clicked_ontrack_int (GtkButton* button, gpointer userdata)
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
	
 		
		if (robot->ecp->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_joints(irp6o_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[0]);
					gtk_entry_set_text(entry1, buf);
					irp6o_desired_pos[0] = irp6o_current_pos[0];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[1]);
					gtk_entry_set_text(entry2, buf);
					irp6o_desired_pos[1] = irp6o_current_pos[1];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[2]);
					gtk_entry_set_text(entry3, buf);
					irp6o_desired_pos[2] = irp6o_current_pos[2];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[3]);
					gtk_entry_set_text(entry4, buf);
					irp6o_desired_pos[3] = irp6o_current_pos[3];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[4]);
					gtk_entry_set_text(entry5, buf);
					irp6o_desired_pos[4] = irp6o_current_pos[4];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[5]);
					gtk_entry_set_text(entry6, buf);
					irp6o_desired_pos[5] = irp6o_current_pos[5];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[6]);
					gtk_entry_set_text(entry7, buf);
					irp6o_desired_pos[6] = irp6o_current_pos[6];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[7]);
					gtk_entry_set_text(entry8, buf);
					irp6o_desired_pos[7] = irp6o_current_pos[7];				
		
 				
 				for (int i = 0; i < 8; i++)
				irp6o_desired_pos[i] = irp6o_current_pos[i];		
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "nie jestem zsynchronizowany" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_ontrack_int (GtkButton* button, gpointer userdata)
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
 	    

		if (robot->ecp->get_EDP_pid()!=-1)
		{
				irp6o_desired_pos[0] = gtk_spin_button_get_value(spin1);
				irp6o_desired_pos[1] = gtk_spin_button_get_value(spin2);
				irp6o_desired_pos[2] = gtk_spin_button_get_value(spin3);
				irp6o_desired_pos[3] = gtk_spin_button_get_value(spin4);
				irp6o_desired_pos[4] = gtk_spin_button_get_value(spin5);
				irp6o_desired_pos[5] = gtk_spin_button_get_value(spin6);
				irp6o_desired_pos[6] = gtk_spin_button_get_value(spin7);
				irp6o_desired_pos[7] = gtk_spin_button_get_value(spin8);
	    
			
			robot->move_joints(irp6o_desired_pos);
			
			 if (state.is_synchronised) {
				gtk_spin_button_set_value(spin1, irp6o_desired_pos[0]);
				gtk_spin_button_set_value(spin2, irp6o_desired_pos[1]);
				gtk_spin_button_set_value(spin3, irp6o_desired_pos[2]);
				gtk_spin_button_set_value(spin4, irp6o_desired_pos[3]);
				gtk_spin_button_set_value(spin5, irp6o_desired_pos[4]);
				gtk_spin_button_set_value(spin6, irp6o_desired_pos[5]);
				gtk_spin_button_set_value(spin7, irp6o_desired_pos[6]);
				gtk_spin_button_set_value(spin8, irp6o_desired_pos[7]);
	  
			 }
		}
		on_read_button_clicked_ontrack_int (button, userdata);

	}
	
	void on_export_button_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
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
 	
 		sprintf(buffer, "edp_irp6o INTERNAL position  %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f" 
 		, gtk_spin_button_get_value(spin1), gtk_spin_button_get_value(spin2), gtk_spin_button_get_value(spin3), gtk_spin_button_get_value(spin4), gtk_spin_button_get_value(spin5), gtk_spin_button_get_value(spin6), gtk_spin_button_get_value(spin7), gtk_spin_button_get_value(spin8));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	

	void on_button1_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}
	
	void on_button2_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}   

	void on_button3_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, gtk_spin_button_get_value(spin2) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}
	
	void on_button4_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, gtk_spin_button_get_value(spin2) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}   

	void on_button5_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, gtk_spin_button_get_value(spin3) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}
	
	void on_button6_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, gtk_spin_button_get_value(spin3) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}   

	void on_button7_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}
	
	void on_button8_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}   

	void on_button9_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}
	
	void on_button10_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}   

	void on_button11_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}
	
	void on_button12_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}   

	void on_button13_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, gtk_spin_button_get_value(spin7) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}
	
	void on_button14_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, gtk_spin_button_get_value(spin7) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}   

	void on_button15_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, gtk_spin_button_get_value(spin8) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}
	
	void on_button16_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, gtk_spin_button_get_value(spin8) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_int (button, userdata);
 	}   

}


extern "C"
{
	void on_arrow_button_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
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
	
	}
	
	void on_read_button_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
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
	
 		
		if (robot->ecp->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_motors(irp6o_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[0]);
					gtk_entry_set_text(entry1, buf);
					irp6o_desired_pos[0] = irp6o_current_pos[0];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[1]);
					gtk_entry_set_text(entry2, buf);
					irp6o_desired_pos[1] = irp6o_current_pos[1];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[2]);
					gtk_entry_set_text(entry3, buf);
					irp6o_desired_pos[2] = irp6o_current_pos[2];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[3]);
					gtk_entry_set_text(entry4, buf);
					irp6o_desired_pos[3] = irp6o_current_pos[3];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[4]);
					gtk_entry_set_text(entry5, buf);
					irp6o_desired_pos[4] = irp6o_current_pos[4];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[5]);
					gtk_entry_set_text(entry6, buf);
					irp6o_desired_pos[5] = irp6o_current_pos[5];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[6]);
					gtk_entry_set_text(entry7, buf);
					irp6o_desired_pos[6] = irp6o_current_pos[6];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos[7]);
					gtk_entry_set_text(entry8, buf);
					irp6o_desired_pos[7] = irp6o_current_pos[7];				
	
 				
 				for (int i = 0; i < 8; i++)
				irp6o_desired_pos[i] = irp6o_current_pos[i];			
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "nie jestem zsynchronizowany" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
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
 	    

		if (robot->ecp->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) {
				irp6o_desired_pos[0] = gtk_spin_button_get_value(spin1);
				irp6o_desired_pos[1] = gtk_spin_button_get_value(spin2);
				irp6o_desired_pos[2] = gtk_spin_button_get_value(spin3);
				irp6o_desired_pos[3] = gtk_spin_button_get_value(spin4);
				irp6o_desired_pos[4] = gtk_spin_button_get_value(spin5);
				irp6o_desired_pos[5] = gtk_spin_button_get_value(spin6);
				irp6o_desired_pos[6] = gtk_spin_button_get_value(spin7);
				irp6o_desired_pos[7] = gtk_spin_button_get_value(spin8);
	    
			} else {
				 for (int i = 0; i < 8; i++)
				 {
		         	 irp6o_desired_pos[i] = 0.0;
	        	 }
	   		 }
			
			robot->move_motors(irp6o_desired_pos);
			
			 if (state.is_synchronised) {
				gtk_spin_button_set_value(spin1, irp6o_desired_pos[0]);
				gtk_spin_button_set_value(spin2, irp6o_desired_pos[1]);
				gtk_spin_button_set_value(spin3, irp6o_desired_pos[2]);
				gtk_spin_button_set_value(spin4, irp6o_desired_pos[3]);
				gtk_spin_button_set_value(spin5, irp6o_desired_pos[4]);
				gtk_spin_button_set_value(spin6, irp6o_desired_pos[5]);
				gtk_spin_button_set_value(spin7, irp6o_desired_pos[6]);
				gtk_spin_button_set_value(spin8, irp6o_desired_pos[7]);
	  
			 }
		}
		on_read_button_clicked_ontrack_inc (button, userdata);

	}
	
	void on_export_button_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
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
 	
 		sprintf(buffer, "edp_irp6o INCREMENT position  %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f" 
 		, gtk_spin_button_get_value(spin1), gtk_spin_button_get_value(spin2), gtk_spin_button_get_value(spin3), gtk_spin_button_get_value(spin4), gtk_spin_button_get_value(spin5), gtk_spin_button_get_value(spin6), gtk_spin_button_get_value(spin7), gtk_spin_button_get_value(spin8));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	
	
	

	void on_button1_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_ontrack_inc (button, userdata); 	
 	}
	
	void on_button2_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_ontrack_inc (button, userdata);
 	}    

	void on_button3_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, gtk_spin_button_get_value(spin2) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_ontrack_inc (button, userdata); 	
 	}
	
	void on_button4_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, gtk_spin_button_get_value(spin2) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_ontrack_inc (button, userdata);
 	}    

	void on_button5_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, gtk_spin_button_get_value(spin3) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_ontrack_inc (button, userdata); 	
 	}
	
	void on_button6_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, gtk_spin_button_get_value(spin3) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_ontrack_inc (button, userdata);
 	}    

	void on_button7_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_ontrack_inc (button, userdata); 	
 	}
	
	void on_button8_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_ontrack_inc (button, userdata);
 	}    

	void on_button9_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_ontrack_inc (button, userdata); 	
 	}
	
	void on_button10_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_ontrack_inc (button, userdata);
 	}    

	void on_button11_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_ontrack_inc (button, userdata); 	
 	}
	
	void on_button12_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_ontrack_inc (button, userdata);
 	}    

	void on_button13_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, gtk_spin_button_get_value(spin7) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_ontrack_inc (button, userdata); 	
 	}
	
	void on_button14_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, gtk_spin_button_get_value(spin7) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_ontrack_inc (button, userdata);
 	}    

	void on_button15_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, gtk_spin_button_get_value(spin8) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_ontrack_inc (button, userdata); 	
 	}
	
	void on_button16_clicked_ontrack_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, gtk_spin_button_get_value(spin8) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_ontrack_inc (button, userdata);
 	}    

}


extern "C"
{
	void on_arrow_button_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
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
	
	}
	
	void on_read_button_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
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
	
 		
		if (robot->ecp->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_xyz_angle_axis(irp6o_current_pos_a))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
				alfa = sqrt(irp6o_current_pos_a[3]*irp6o_current_pos_a[3]
				+irp6o_current_pos_a[4]*irp6o_current_pos_a[4]
				+irp6o_current_pos_a[5]*irp6o_current_pos_a[5]);
					
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_a[0]);
					gtk_entry_set_text(entry1, buf);
					irp6o_desired_pos_a[0] = irp6o_current_pos_a[0];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_a[1]);
					gtk_entry_set_text(entry2, buf);
					irp6o_desired_pos_a[1] = irp6o_current_pos_a[1];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_a[2]);
					gtk_entry_set_text(entry3, buf);
					irp6o_desired_pos_a[2] = irp6o_current_pos_a[2];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_a[3]/alfa);
					gtk_entry_set_text(entry4, buf);
					irp6o_desired_pos_a[3] = irp6o_current_pos_a[3]/alfa;
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_a[4]/alfa);
					gtk_entry_set_text(entry5, buf);
					irp6o_desired_pos_a[4] = irp6o_current_pos_a[4]/alfa;
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_a[5]/alfa);
					gtk_entry_set_text(entry6, buf);
					irp6o_desired_pos_a[5] = irp6o_current_pos_a[5]/alfa;
					snprintf (buf, sizeof(buf), "%.3f", alfa);
					gtk_entry_set_text(entry7, buf);
					irp6o_desired_pos_a[6] = alfa;							
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_a[7]);
					gtk_entry_set_text(entry8, buf);
					irp6o_desired_pos_a[7] = irp6o_current_pos_a[7];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_a[8]);
					gtk_entry_set_text(entry9, buf);
					irp6o_desired_pos_a[8] = irp6o_current_pos_a[8];				
				
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "nie jestem zsynchronizowany" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
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
 	    

		if (robot->ecp->get_EDP_pid()!=-1)
		{
				irp6o_desired_pos_a[0] = gtk_spin_button_get_value(spin1);
				irp6o_desired_pos_a[1] = gtk_spin_button_get_value(spin2);
				irp6o_desired_pos_a[2] = gtk_spin_button_get_value(spin3);
				irp6o_desired_pos_a[3] = gtk_spin_button_get_value(spin4);
				irp6o_desired_pos_a[4] = gtk_spin_button_get_value(spin5);
				irp6o_desired_pos_a[5] = gtk_spin_button_get_value(spin6);
				irp6o_desired_pos_a[6] = gtk_spin_button_get_value(spin7);
				irp6o_desired_pos_a[7] = gtk_spin_button_get_value(spin8);
				irp6o_desired_pos_a[8] = gtk_spin_button_get_value(spin9);
	    
 		
 			// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
			for(int i=3; i<9; i++)
			{
					irp6o_desired_pos_a[i] *= irp6o_desired_pos_a[6];
			}
			
			robot->move_xyz_angle_axis(irp6o_desired_pos_a);
			
			 if (state.is_synchronised) {
				gtk_spin_button_set_value(spin1, irp6o_desired_pos_a[0]);
				gtk_spin_button_set_value(spin2, irp6o_desired_pos_a[1]);
				gtk_spin_button_set_value(spin3, irp6o_desired_pos_a[2]);
				gtk_spin_button_set_value(spin4, irp6o_desired_pos_a[3]);
				gtk_spin_button_set_value(spin5, irp6o_desired_pos_a[4]);
				gtk_spin_button_set_value(spin6, irp6o_desired_pos_a[5]);
				gtk_spin_button_set_value(spin7, irp6o_desired_pos_a[6]);
				gtk_spin_button_set_value(spin8, irp6o_desired_pos_a[7]);
				gtk_spin_button_set_value(spin9, irp6o_desired_pos_a[8]);
	  
			 }
		}
		on_read_button_clicked_ontrack_axis_xyz (button, userdata);

	}
	

	void on_button1_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) - gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
 	}
	
	void on_button2_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) + gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
}   

	void on_button3_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, gtk_spin_button_get_value(spin2) - gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
 	}
	
	void on_button4_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, gtk_spin_button_get_value(spin2) + gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
}   

	void on_button5_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, gtk_spin_button_get_value(spin3) - gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
 	}
	
	void on_button6_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, gtk_spin_button_get_value(spin3) + gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
}   

	void on_button7_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
               
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}
	
	void on_button8_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}  

	void on_button9_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
               
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}
	
	void on_button10_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}  

	void on_button11_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
               
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}
	
	void on_button12_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}  

	void on_button13_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, gtk_spin_button_get_value(spin7) - gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}
	
	void on_button14_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, gtk_spin_button_get_value(spin7) + gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}   

	void on_button15_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, gtk_spin_button_get_value(spin8) - gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}
	
	void on_button16_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, gtk_spin_button_get_value(spin8) + gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}   

	void on_button17_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin9 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton9"));
        gtk_spin_button_set_value(spin9, gtk_spin_button_get_value(spin9) - gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}
	
	void on_button18_clicked_ontrack_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin9 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton9"));
        gtk_spin_button_set_value(spin9, gtk_spin_button_get_value(spin9) + gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
		
		on_execute_button_clicked_ontrack_axis_xyz (button, userdata);
	}   

}


extern "C"
{
	void on_arrow_button_clicked_ontrack_axis_ts (GtkButton* button, gpointer userdata)
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
	
 	}
	
	void on_read_button_clicked_ontrack_axis_ts (GtkButton* button, gpointer userdata)
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
	
 		
		if (robot->ecp->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_tool_xyz_angle_axis(tool_vector_a))) // Odczyt polozenia walow silnikow
					printf("Blad w read external\n");
					
				alfa = sqrt(tool_vector_a[3]*tool_vector_a[3]
				+tool_vector_a[4]*tool_vector_a[4]
				+tool_vector_a[5]*tool_vector_a[5]);
				
				if (alfa==0){
					tool_vector_a[3] = -1;
					tool_vector_a[4] = 0;
					tool_vector_a[5] = 0;
				}
				else{
					tool_vector_a[3] = tool_vector_a[3]/alfa;
					tool_vector_a[4] = tool_vector_a[4]/alfa;
					tool_vector_a[5] = tool_vector_a[5]/alfa;
				}
					
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[0]);
					gtk_entry_set_text(entry1, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[1]);
					gtk_entry_set_text(entry2, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[2]);
					gtk_entry_set_text(entry3, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[3]);
					gtk_entry_set_text(entry4, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[4]);
					gtk_entry_set_text(entry5, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[5]);
					gtk_entry_set_text(entry6, buf);
					snprintf (buf, sizeof(buf), "%.3f", alfa);
					gtk_entry_set_text(entry7, buf);
				
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "nie jestem zsynchronizowany" << std::endl;
			}
		}
	
	}
	
	void on_set_button_clicked_ontrack_axis_ts (GtkButton* button, gpointer userdata)
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
 	    

		if (state.is_synchronised)
		{
				tool_vector_a[0] = gtk_spin_button_get_value(spin1);
				tool_vector_a[1] = gtk_spin_button_get_value(spin2);
				tool_vector_a[2] = gtk_spin_button_get_value(spin3);
				tool_vector_a[3] = gtk_spin_button_get_value(spin4);
				tool_vector_a[4] = gtk_spin_button_get_value(spin5);
				tool_vector_a[5] = gtk_spin_button_get_value(spin6);
				tool_vector_a[6] = gtk_spin_button_get_value(spin7);
	    
 		
 		wl = sqrt(tool_vector_a[3]*tool_vector_a[3] + tool_vector_a[4]*tool_vector_a[4] + tool_vector_a[5]*tool_vector_a[5]);

		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			tool_vector_a[3] = tool_vector_a[3]/wl;
			tool_vector_a[4] = tool_vector_a[4]/wl;
			tool_vector_a[5] = tool_vector_a[5]/wl;
		}
		
		for(int i=3; i<7; i++)
		{
				tool_vector_a[i] *= tool_vector_a[6];
		}
		
			robot->set_tool_xyz_angle_axis(tool_vector_a);		
		}
		on_read_button_clicked_ontrack_axis_ts (button, userdata);

	}

}


extern "C"
{
	void on_arrow_button_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
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
	
	}
	
	void on_read_button_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
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
	
 		
		if (robot->ecp->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_xyz_euler_zyz(irp6o_current_pos_e))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_e[0]);
					gtk_entry_set_text(entry1, buf);
					irp6o_desired_pos_e[0] = irp6o_current_pos_e[0];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_e[1]);
					gtk_entry_set_text(entry2, buf);
					irp6o_desired_pos_e[1] = irp6o_current_pos_e[1];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_e[2]);
					gtk_entry_set_text(entry3, buf);
					irp6o_desired_pos_e[2] = irp6o_current_pos_e[2];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_e[3]);
					gtk_entry_set_text(entry4, buf);
					irp6o_desired_pos_e[3] = irp6o_current_pos_e[3];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_e[4]);
					gtk_entry_set_text(entry5, buf);
					irp6o_desired_pos_e[4] = irp6o_current_pos_e[4];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_e[5]);
					gtk_entry_set_text(entry6, buf);
					irp6o_desired_pos_e[5] = irp6o_current_pos_e[5];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_e[6]);
					gtk_entry_set_text(entry7, buf);
					irp6o_desired_pos_e[6] = irp6o_current_pos_e[6];				
					snprintf (buf, sizeof(buf), "%.3f", irp6o_current_pos_e[7]);
					gtk_entry_set_text(entry8, buf);
					irp6o_desired_pos_e[7] = irp6o_current_pos_e[7];				
		
 				
				for (int i = 0; i < 8; i++)
				irp6o_desired_pos_e[i] = irp6o_current_pos_e[i];		
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "nie jestem zsynchronizowany" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
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
 	    

		if (robot->ecp->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) {
				irp6o_desired_pos_e[0] = gtk_spin_button_get_value(spin1);
				irp6o_desired_pos_e[1] = gtk_spin_button_get_value(spin2);
				irp6o_desired_pos_e[2] = gtk_spin_button_get_value(spin3);
				irp6o_desired_pos_e[3] = gtk_spin_button_get_value(spin4);
				irp6o_desired_pos_e[4] = gtk_spin_button_get_value(spin5);
				irp6o_desired_pos_e[5] = gtk_spin_button_get_value(spin6);
				irp6o_desired_pos_e[6] = gtk_spin_button_get_value(spin7);
				irp6o_desired_pos_e[7] = gtk_spin_button_get_value(spin8);
	    
			
			robot->move_xyz_euler_zyz(irp6o_desired_pos_e);
			}
			 if (state.is_synchronised) {
				gtk_spin_button_set_value(spin1, irp6o_desired_pos_e[0]);
				gtk_spin_button_set_value(spin2, irp6o_desired_pos_e[1]);
				gtk_spin_button_set_value(spin3, irp6o_desired_pos_e[2]);
				gtk_spin_button_set_value(spin4, irp6o_desired_pos_e[3]);
				gtk_spin_button_set_value(spin5, irp6o_desired_pos_e[4]);
				gtk_spin_button_set_value(spin6, irp6o_desired_pos_e[5]);
				gtk_spin_button_set_value(spin7, irp6o_desired_pos_e[6]);
				gtk_spin_button_set_value(spin8, irp6o_desired_pos_e[7]);
	  
			 }
		}
		on_read_button_clicked_ontrack_euler_xyz (button, userdata);

	}
	
	void on_export_button_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
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
 	
 		sprintf(buffer, "edp_irp6o XYZ EULER ZYZ position  %f %f %f %f %f %f %f %f" 
 		, gtk_spin_button_get_value(spin1), gtk_spin_button_get_value(spin2), gtk_spin_button_get_value(spin3), gtk_spin_button_get_value(spin4), gtk_spin_button_get_value(spin5), gtk_spin_button_get_value(spin6), gtk_spin_button_get_value(spin7), gtk_spin_button_get_value(spin8));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	

	void on_button1_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	} 
	
	void on_button2_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	}   

	void on_button3_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, gtk_spin_button_get_value(spin2) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	} 
	
	void on_button4_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, gtk_spin_button_get_value(spin2) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	}   

	void on_button5_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, gtk_spin_button_get_value(spin3) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	} 
	
	void on_button6_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
        gtk_spin_button_set_value(spin3, gtk_spin_button_get_value(spin3) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	}   

	void on_button7_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	} 
	
	void on_button8_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
        gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	}   

	void on_button9_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	} 
	
	void on_button10_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
        gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	}   

	void on_button11_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	} 
	
	void on_button12_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
        gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	}   

	void on_button13_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, gtk_spin_button_get_value(spin7) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	} 
	
	void on_button14_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin7 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7"));
        gtk_spin_button_set_value(spin7, gtk_spin_button_get_value(spin7) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	}   

	void on_button15_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, gtk_spin_button_get_value(spin8) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	} 
	
	void on_button16_clicked_ontrack_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin8 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8"));
        gtk_spin_button_set_value(spin8, gtk_spin_button_get_value(spin8) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_ontrack_euler_xyz (button, userdata);
 	}   

}


extern "C"
{
	void on_arrow_button_clicked_ontrack_euler_ts (GtkButton* button, gpointer userdata)
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
		
 	}
	
	void on_read_button_clicked_ontrack_euler_ts (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
		GtkEntry * entry2 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2"));
		GtkEntry * entry3 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3"));
		GtkEntry * entry4 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4"));
		GtkEntry * entry5 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5"));
		GtkEntry * entry6 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6"));
	
 		
		if (robot->ecp->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_tool_xyz_euler_zyz(tool_vector_e))) // Odczyt polozenia walow silnikow
					printf("Blad w read external\n");
					
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[0]);
					gtk_entry_set_text(entry1, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[1]);
					gtk_entry_set_text(entry2, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[2]);
					gtk_entry_set_text(entry3, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[3]);
					gtk_entry_set_text(entry4, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[4]);
					gtk_entry_set_text(entry5, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[5]);
					gtk_entry_set_text(entry6, buf);		
				
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "nie jestem zsynchronizowany" << std::endl;
			}
		}
	
	}
	
	void on_set_button_clicked_ontrack_euler_ts (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
 		GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
 		GtkSpinButton * spin3 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3"));
 		GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4"));
 		GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5"));
 		GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6"));
 	    

		if (state.is_synchronised)
		{
				tool_vector_e[0] = gtk_spin_button_get_value(spin1);
				tool_vector_e[1] = gtk_spin_button_get_value(spin2);
				tool_vector_e[2] = gtk_spin_button_get_value(spin3);
				tool_vector_e[3] = gtk_spin_button_get_value(spin4);
				tool_vector_e[4] = gtk_spin_button_get_value(spin5);
				tool_vector_e[5] = gtk_spin_button_get_value(spin6);
	    
			
			robot->set_tool_xyz_euler_zyz(tool_vector_e);
		}
		on_read_button_clicked_ontrack_euler_ts (button, userdata);

	}
}

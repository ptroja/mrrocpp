
#include <iostream>

#include <gtk/gtk.h>
#include <glib.h>

#include "ui_model.h"
#include "edp_conveyor_uimodule.h"


lib::BYTE servo_alg_no[1];
lib::BYTE servo_par_no[1];
	
gint servo_alg_no_tmp [1];
lib::BYTE servo_alg_no_output[1];
gint servo_par_no_tmp [1];
lib::BYTE servo_par_no_output[1];

char buf[32];
gchar buffer[500];
double tool_vector_a[0];
double tool_vector_e[0];
double alfa, kx, ky, kz;
double wl; 
double l_eps = 0;
double conveyor_current_pos_a[0]; // pozycja biezaca
double conveyor_desired_pos_a[0]; // pozycja zadana
double conveyor_current_pos_e[0]; // pozycja biezaca
double conveyor_desired_pos_e[0]; // pozycja zadana
double conveyor_current_pos[1]; // pozycja biezaca
double conveyor_desired_pos[1]; // pozycja zadana



#include "ui/ui_ecp_r_conveyor.h"

edp_conveyor::edp_conveyor(ui_config_entry &entry)
{
				robot = new ui_conveyor_robot(
				ui_model::instance().getConfigurator(),
				&ui_model::instance().getEcpSr()
				
				);
				
				robot->get_controller_state(&state);
}

edp_conveyor::~edp_conveyor()
{
	if (robot) {
		delete robot;
	}		
}

static edp_conveyor *edp_conveyorRobot;


extern "C" 
{ 
	void  on_combobox1_changed_conveyorRobot(GtkComboBox *comboBox, gpointer userdata)  
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
		case 3:  break;
		case 4:  break;
		case 5:  break;
		case 6:  break;
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
		edp_conveyorRobot = new edp_conveyor(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (edp_conveyorRobot) 
		{
			delete edp_conveyorRobot;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}


extern "C"
{
	void on_arrow_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entry1)));
	
        GtkEntry * entry2 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2"));
        GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));
        gtk_spin_button_set_value(spin2, atof(gtk_entry_get_text(entry2)));
	
	}
	
	void on_read_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
		GtkEntry * entry2 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2"));


		if (robot->get_EDP_pid()!=-1)
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
	
	void on_set_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());

		GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
		GtkSpinButton * spin2 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2"));

 		
 		if (state.is_synchronised)
		{
			servo_alg_no_tmp[0] = gtk_spin_button_get_value_as_int(spin1);
			servo_par_no_tmp[0] = gtk_spin_button_get_value_as_int(spin2);


		for(int i=0; i<1; i++)
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
	void on_arrow_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entry1)));
	
	}
	
	void on_read_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
	
 		
		if (robot->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_joints(conveyor_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", conveyor_current_pos[0]);
					gtk_entry_set_text(entry1, buf);
					conveyor_desired_pos[0] = conveyor_current_pos[0];				
		
 				
 				for (int i = 0; i < 1; i++)
				conveyor_desired_pos[i] = conveyor_current_pos[i];		
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "nie jestem zsynchronizowany" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
 	    

		if (robot->get_EDP_pid()!=-1)
		{
				conveyor_desired_pos[0] = gtk_spin_button_get_value(spin1);
	    
			
			robot->move_joints(conveyor_desired_pos);
			
			 if (state.is_synchronised) {
				gtk_spin_button_set_value(spin1, conveyor_desired_pos[0]);
	  
			 }
		}
		on_read_button_clicked_conveyorRobot_int (button, userdata);

	}
	
	void on_export_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
 	
 		sprintf(buffer, "edp_conveyor INTERNAL position  %.3f" 
 		, gtk_spin_button_get_value(spin1));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	

	void on_button1_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_conveyorRobot_int (button, userdata);
 	}
	
	void on_button2_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_conveyorRobot_int (button, userdata);
 	}   

}


extern "C"
{
	void on_arrow_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entry1)));
	
	}
	
	void on_read_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1 = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1"));
	
 		
		if (robot->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_motors(conveyor_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", conveyor_current_pos[0]);
					gtk_entry_set_text(entry1, buf);
					conveyor_desired_pos[0] = conveyor_current_pos[0];				
	
 				
 				for (int i = 0; i < 1; i++)
				conveyor_desired_pos[i] = conveyor_current_pos[i];			
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "nie jestem zsynchronizowany" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
 	    

		if (robot->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) {
				conveyor_desired_pos[0] = gtk_spin_button_get_value(spin1);
	    
			} else {
				 for (int i = 0; i < 1; i++)
				 {
		         	 conveyor_desired_pos[i] = 0.0;
	        	 }
	   		 }
			
			robot->move_motors(conveyor_desired_pos);
			
			 if (state.is_synchronised) {
				gtk_spin_button_set_value(spin1, conveyor_desired_pos[0]);
	  
			 }
		}
		on_read_button_clicked_conveyorRobot_inc (button, userdata);

	}
	
	void on_export_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
 	
 		sprintf(buffer, "edp_conveyor INCREMENT position  %.3f" 
 		, gtk_spin_button_get_value(spin1));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	
	
	

	void on_button1_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) - gtk_spin_button_get_value(spinbuttonDown1));
 	
		on_execute_button_clicked_conveyorRobot_inc (button, userdata); 	
 	}
	
	void on_button2_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1"));
        gtk_spin_button_set_value(spin1, gtk_spin_button_get_value(spin1) + gtk_spin_button_get_value(spinbuttonDown1));
 	
 		on_execute_button_clicked_conveyorRobot_inc (button, userdata);
 	}    

}

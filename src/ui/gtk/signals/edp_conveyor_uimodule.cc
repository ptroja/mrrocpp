
#include <iostream>

#include <gtk/gtk.h>
#include <glib.h>

#include "ui_model.h"
#include "edp_conveyor_uimodule.h"


mrrocpp::lib::BYTE servo_alg_no[1];
mrrocpp::lib::BYTE servo_par_no[1];
	
gint servo_alg_no_tmp [1];
mrrocpp::lib::BYTE servo_alg_no_output[1];
gint servo_par_no_tmp [1];
mrrocpp::lib::BYTE servo_par_no_output[1];

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
				robot_conveyorRobot = new ui_conveyor_robot(
				ui_model::instance().getConfigurator(),
				&ui_model::instance().getEcpSr()
				
				);

}

edp_conveyor::~edp_conveyor()
{
	if (robot_conveyorRobot) {
		delete robot_conveyorRobot;
	}		
}

static edp_conveyor *edp_conveyorRobot;


extern "C" 
{ 
	void on_read_button_clicked_conveyorRobot_servo (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_conveyorRobot_int (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_conveyorRobot_inc (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_conveyorRobot_axis_xyz (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_conveyorRobot_euler_xyz (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_conveyorRobot_axis_ts (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_conveyorRobot_euler_ts (GtkButton * button, gpointer userdata);


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

		if (state_conveyorRobot.is_synchronised)
		{
			switch (choice)
			{
			case 0: std::cout << "Internal window chosen" << std::endl; isFile = 1; windowName = "window_int"; on_read_button_clicked_conveyorRobot_int (button, userdata); break;
			case 1: std::cout << "Increment window chosen" << std::endl; isFile = 1; windowName = "window_inc"; on_read_button_clicked_conveyorRobot_inc (button, userdata); break;
			case 2: std::cout << "Servo algorithm window chosen" << std::endl; isFile = 1; windowName = "window_servo"; on_read_button_clicked_conveyorRobot_servo (button, userdata); break;
			case 3:  on_read_button_clicked_conveyorRobot_axis_xyz (button, userdata); break;
			case 4:  on_read_button_clicked_conveyorRobot_euler_xyz (button, userdata); break;
			case 5:  on_read_button_clicked_conveyorRobot_axis_ts (button, userdata); break;
			case 6:  on_read_button_clicked_conveyorRobot_euler_ts (button, userdata); break;
			default: std::cout << "Synchronizing..." << std::endl;
			}
		}
		else
		{
			switch (choice)
			{
			case 0: std::cout << "Internal window chosen" << std::endl; isFile = 1; windowName = "window_int"; on_read_button_clicked_conveyorRobot_int (button, userdata); break;
			case 1: std::cout << "Increment window chosen" << std::endl; isFile = 1; windowName = "window_inc"; on_read_button_clicked_conveyorRobot_inc (button, userdata); break;
			case 2: break;
			case 3: break;
			case 4: break;
			case 5: break;
			case 6: break;
			default: ;
			}
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
	
	void  on_clicked_synchronize_conveyorRobot(GtkButton * button, gpointer userdata)  
	{
		ui_config_entry & comboEntry = *(ui_config_entry *) userdata;
		GtkBuilder & builder = (comboEntry.getBuilder());
		gint counter_synch;
		
		robot_conveyorRobot->get_controller_state (&state_conveyorRobot);
	        if(!state_conveyorRobot.is_synchronised) {
	   	        GThread * synchronization_thread_conveyorRobot = g_thread_create(ui_synchronize_conveyorRobot, userdata, false, &error);
	        	if (synchronization_thread_conveyorRobot == NULL) 
	     		{
	        		fprintf(stderr, "g_thread_create(): %s\n", error->message);
	        	}
	      }
	        robot_conveyorRobot->get_controller_state (&state_conveyorRobot);
	        if (state_conveyorRobot.is_synchronised) {
	            gtk_widget_set_sensitive( GTK_WIDGET(button), FALSE);
	            
	            	GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&builder, "combobox1"));

					counter_synch = 2;
					gtk_combo_box_insert_text(combo, counter_synch, "Servo algorithm"); counter_synch++;
					
					
					
					

	        }
	}	

	void ui_module_init(ui_config_entry &entry) 
	{
		edp_conveyorRobot = new edp_conveyor(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);

		gint counter = 0;
		GtkBuilder & builder = (entry.getBuilder());
		GtkButton * button = GTK_BUTTON (gtk_builder_get_object(&builder, "button_synchronize"));
		if (state_conveyorRobot.is_synchronised) gtk_widget_set_sensitive( GTK_WIDGET(button), FALSE);
		else
		{
			GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&builder, "combobox1"));
			
			gtk_combo_box_remove_text(combo, counter); gtk_combo_box_insert_text(combo, counter, "Internal"); counter++; gtk_combo_box_insert_text(combo, counter, "Increment"); counter++;
		}
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

void *ui_synchronize_conveyorRobot (gpointer userdata)
{
	ui_config_entry & comboEntry = *(ui_config_entry *) userdata;
	GtkBuilder & builder = (comboEntry.getBuilder());
	gint counter = 0;

	robot_conveyorRobot->synchronise();
    
	GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&builder, "combobox1"));

	counter = 2;
	gtk_combo_box_insert_text(combo, counter, "Servo algorithm"); counter++;
	
	
	
	
	return NULL;
}



extern "C"
{
	void on_arrow_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_servo_conveyor = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_servo_conveyor"));
        GtkSpinButton * spin1_servo_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_servo_conveyor"));
        gtk_spin_button_set_value(spin1_servo_conveyor, atof(gtk_entry_get_text(entry1_servo_conveyor)));
	
        GtkEntry * entry2_servo_conveyor = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_servo_conveyor"));
        GtkSpinButton * spin2_servo_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_servo_conveyor"));
        gtk_spin_button_set_value(spin2_servo_conveyor, atof(gtk_entry_get_text(entry2_servo_conveyor)));
	
	}
	
	void on_read_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_servo_conveyor = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_servo_conveyor"));
		GtkEntry * entry2_servo_conveyor = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_servo_conveyor"));


		if (robot_conveyorRobot->get_EDP_pid()!=-1)
		{
				if (state_conveyorRobot.is_synchronised)  // Czy robot jest zsynchronizowany?
				{
					if (!(robot_conveyorRobot->get_servo_algorithm(servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
						printf("Blad w mechatronika get_servo_algorithm\n");
					
					gtk_entry_set_text(entry1_servo_conveyor, (const gchar*)servo_alg_no[0]);
					gtk_entry_set_text(entry2_servo_conveyor, (const gchar*)servo_par_no[0]);	
					
				} else
				{
					std::cout << "Robot is not synchronized" << std::endl;
				}
			}
	}
	
	void on_set_button_clicked_conveyorRobot_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());

		GtkSpinButton * spin1_servo_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_servo_conveyor"));
		GtkSpinButton * spin2_servo_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_servo_conveyor"));

 		
 		if (state_conveyorRobot.is_synchronised)
		{
			servo_alg_no_tmp[0] = gtk_spin_button_get_value_as_int(spin1_servo_conveyor);
			servo_par_no_tmp[0] = gtk_spin_button_get_value_as_int(spin2_servo_conveyor);


		for(int i=0; i<1; i++)
		{
			servo_alg_no_output[i] = mrrocpp::lib::BYTE(servo_alg_no_tmp[i]);
			servo_par_no_output[i] = mrrocpp::lib::BYTE(servo_par_no_tmp[i]);
		}

		// zlecenie wykonania ruchu
		robot_conveyorRobot->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

	}
	else
	{
		std::cout << "Robot is not synchronized" << std::endl;
	}
 		
	}
}


extern "C"
{
	void on_arrow_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_int_conveyor = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_int_conveyor"));
        GtkSpinButton * spin1_int_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_conveyor"));
        gtk_spin_button_set_value(spin1_int_conveyor, atof(gtk_entry_get_text(entry1_int_conveyor)));
	
	}
	
	void on_read_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_int_conveyor = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_int_conveyor"));
	
 		
		if (robot_conveyorRobot->get_EDP_pid()!=-1)
		{
			if (state_conveyorRobot.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_conveyorRobot->read_joints(conveyor_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", conveyor_current_pos[0]);
					gtk_entry_set_text(entry1_int_conveyor, buf);
					conveyor_desired_pos[0] = conveyor_current_pos[0];				
		
 				
 				for (int i = 0; i < 1; i++)
				conveyor_desired_pos[i] = conveyor_current_pos[i];		
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "Robot is not synchronized" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_int_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_conveyor"));
 	    

		if (robot_conveyorRobot->get_EDP_pid()!=-1)
		{
				conveyor_desired_pos[0] = gtk_spin_button_get_value(spin1_int_conveyor);
	    
			
			robot_conveyorRobot->move_joints(conveyor_desired_pos);
			
			 if (state_conveyorRobot.is_synchronised) {
				gtk_spin_button_set_value(spin1_int_conveyor, conveyor_desired_pos[0]);
	  
			 }
		}
		on_read_button_clicked_conveyorRobot_int (button, userdata);

	}
	
	void on_export_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_int_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_conveyor"));
 	
 		sprintf(buffer, "edp_conveyor INTERNAL position  %.3f" 
 		, gtk_spin_button_get_value(spin1_int_conveyor));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1_int_ = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_"));
        gtk_spin_button_set_value(spin1_int_, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	

	void on_button1_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_conveyor"));
        GtkSpinButton * spin1_int_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_conveyor"));
        gtk_spin_button_set_value(spin1_int_conveyor, gtk_spin_button_get_value(spin1_int_conveyor) - gtk_spin_button_get_value(spinbuttonDown1_int_conveyor));
 		
 		on_execute_button_clicked_conveyorRobot_int (button, userdata);
 	}
	
	void on_button2_clicked_conveyorRobot_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_conveyor"));
        GtkSpinButton * spin1_int_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_conveyor"));
        gtk_spin_button_set_value(spin1_int_conveyor, gtk_spin_button_get_value(spin1_int_conveyor) + gtk_spin_button_get_value(spinbuttonDown1_int_conveyor));
 		
 		on_execute_button_clicked_conveyorRobot_int (button, userdata);
 	}   

}


extern "C"
{
	void on_arrow_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_inc_conveyor = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_inc_conveyor"));
        GtkSpinButton * spin1_inc_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_conveyor"));
        gtk_spin_button_set_value(spin1_inc_conveyor, atof(gtk_entry_get_text(entry1_inc_conveyor)));
	
	}
	
	void on_read_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_inc_conveyor = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_inc_conveyor"));
	
 		
		if (robot_conveyorRobot->get_EDP_pid()!=-1)
		{
			if (state_conveyorRobot.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_conveyorRobot->read_motors(conveyor_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", conveyor_current_pos[0]);
					gtk_entry_set_text(entry1_inc_conveyor, buf);
					conveyor_desired_pos[0] = conveyor_current_pos[0];				
	
 				
 				for (int i = 0; i < 1; i++)
				conveyor_desired_pos[i] = conveyor_current_pos[i];			
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "Robot is not synchronized" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_inc_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_conveyor"));
 	    

		if (robot_conveyorRobot->get_EDP_pid()!=-1)
		{
			if (state_conveyorRobot.is_synchronised) {
				conveyor_desired_pos[0] = gtk_spin_button_get_value(spin1_inc_conveyor);
	    
			} else {
				 for (int i = 0; i < 1; i++)
				 {
		         	 conveyor_desired_pos[i] = 0.0;
	        	 }
	   		 }
			
			robot_conveyorRobot->move_motors(conveyor_desired_pos);
			
			 if (state_conveyorRobot.is_synchronised) {
				gtk_spin_button_set_value(spin1_inc_conveyor, conveyor_desired_pos[0]);
	  
			 }
		}
		on_read_button_clicked_conveyorRobot_inc (button, userdata);

	}
	
	void on_export_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_inc_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_conveyor"));
 	
 		sprintf(buffer, "edp_conveyor INCREMENT position  %.3f" 
 		, gtk_spin_button_get_value(spin1_inc_conveyor));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1_inc_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_conveyor"));
        gtk_spin_button_set_value(spin1_inc_conveyor, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	
	
	

	void on_button1_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_conveyor"));
        GtkSpinButton * spin1_inc_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_conveyor"));
        gtk_spin_button_set_value(spin1_inc_conveyor, gtk_spin_button_get_value(spin1_inc_conveyor) - gtk_spin_button_get_value(spinbuttonDown1_inc_conveyor));
 	
		on_execute_button_clicked_conveyorRobot_inc (button, userdata); 	
 	}
	
	void on_button2_clicked_conveyorRobot_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_conveyor"));
        GtkSpinButton * spin1_inc_conveyor = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_conveyor"));
        gtk_spin_button_set_value(spin1_inc_conveyor, gtk_spin_button_get_value(spin1_inc_conveyor) + gtk_spin_button_get_value(spinbuttonDown1_inc_conveyor));
 	
 		on_execute_button_clicked_conveyorRobot_inc (button, userdata);
 	}    

}


#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6o_int_uimodule.h"


edp_irp6o_int::edp_irp6o_int(ui_config_entry &entry) 
{
}

static edp_irp6o_int *int_ontrack;


extern "C"
{
	void on_arrow_button_clicked_ontrack_int (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla ontrack int" << std::endl;
	}
	
	void on_read_button_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla ontrack int" << std::endl;
	}
	
	void on_execute_button_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Execute move dla ontrack int" << std::endl;
	}
	
	void on_export_button_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Export dla ontrack int" << std::endl;
	}
	
	void on_import_button_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Import dla ontrack int" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		int_ontrack = new edp_irp6o_int(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (int_ontrack) 
		{
			delete int_ontrack;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
	

	void on_button1_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button1 dla ontrack int" << std::endl;
	}
    

	void on_button2_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button2 dla ontrack int" << std::endl;
	}
    

	void on_button3_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button3 dla ontrack int" << std::endl;
	}
    

	void on_button4_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button4 dla ontrack int" << std::endl;
	}
    

	void on_button5_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button5 dla ontrack int" << std::endl;
	}
    

	void on_button6_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button6 dla ontrack int" << std::endl;
	}
    

	void on_button7_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button7 dla ontrack int" << std::endl;
	}
    

	void on_button8_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button8 dla ontrack int" << std::endl;
	}
    

	void on_button9_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button9 dla ontrack int" << std::endl;
	}
    

	void on_button10_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button10 dla ontrack int" << std::endl;
	}
    

	void on_button11_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button11 dla ontrack int" << std::endl;
	}
    

	void on_button12_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button12 dla ontrack int" << std::endl;
	}
    

	void on_button13_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button13 dla ontrack int" << std::endl;
	}
    

	void on_button14_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button14 dla ontrack int" << std::endl;
	}
    

	void on_button15_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button15 dla ontrack int" << std::endl;
	}
    

	void on_button16_clicked_ontrack_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button16 dla ontrack int" << std::endl;
	}
    

}

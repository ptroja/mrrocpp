
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "irp6p_int_uimodule.h"


edp_irp6p_int::edp_irp6p_int(ui_config_entry &entry) 
{
}

static edp_irp6p_int *int_postument;


extern "C"
{
	void on_arrow_button_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
		std::cout << "skopiuj wartosci dla postument int" << std::endl;
	}
	
	void on_read_button_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "wczytaj wartosci dla postument int" << std::endl;
	}
	
	void on_execute_button_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Execute move dla postument int" << std::endl;
	}
	
	void on_export_button_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Export dla postument int" << std::endl;
	}
	
	void on_import_button_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Import dla postument int" << std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &entry) 
	{
		int_postument = new edp_irp6p_int(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (int_postument) 
		{
			delete int_postument;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
	

	void on_button1_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button1 dla postument int" << std::endl;
	}
    

	void on_button2_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button2 dla postument int" << std::endl;
	}
    

	void on_button3_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button3 dla postument int" << std::endl;
	}
    

	void on_button4_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button4 dla postument int" << std::endl;
	}
    

	void on_button5_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button5 dla postument int" << std::endl;
	}
    

	void on_button6_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button6 dla postument int" << std::endl;
	}
    

	void on_button7_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button7 dla postument int" << std::endl;
	}
    

	void on_button8_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button8 dla postument int" << std::endl;
	}
    

	void on_button9_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button9 dla postument int" << std::endl;
	}
    

	void on_button10_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button10 dla postument int" << std::endl;
	}
    

	void on_button11_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button11 dla postument int" << std::endl;
	}
    

	void on_button12_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button12 dla postument int" << std::endl;
	}
    

	void on_button13_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button13 dla postument int" << std::endl;
	}
    

	void on_button14_clicked_postument_int (GtkButton* button, gpointer user_data)
	{
		std::cout << "Button14 dla postument int" << std::endl;
	}
    

}

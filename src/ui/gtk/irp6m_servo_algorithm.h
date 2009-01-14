#ifndef __EDP_IRP6_SERVO_ALGORITHM
#define __EDP_IRP6_SERVO_ALGORITHM

#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>

class edp_irp6m_servo_algorithm
{
	public:

		edp_irp6m_servo_algorithm(ui_config_entry &entry);
		edp_irp6m_servo_algorithm();
		~edp_irp6m_servo_algorithm();
		void copy_data();

	private:
		GtkButton* arrow;
		GtkButton* button;
};


#endif /* __EDP_IRP6_SERVO_ALGORITHM */

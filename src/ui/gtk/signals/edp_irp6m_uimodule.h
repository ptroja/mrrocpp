
#ifndef __EDP_irp6m
#define __EDP_irp6m

#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>

#include "ui/ui_ecp_r_irp6_common.h"
#include "ui/ui_ecp_r_conveyor.h"

class edp_irp6m
{
	public:

		edp_irp6m(ui_config_entry &entry);
		~edp_irp6m();
};
ui_common_robot * robot_mechatronika;
mrrocpp::lib::controller_state_t state_mechatronika;
GError *error = NULL;
void *ui_synchronize_mechatronika (gpointer userdata);
GtkButton* button;

#endif /* __EDP_irp6m */

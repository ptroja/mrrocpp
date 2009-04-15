
#ifndef __EDP_irp6o
#define __EDP_irp6o

#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>

#include "ui/ui_ecp_r_irp6_common.h"
#include "ui/ui_ecp_r_conveyor.h"

class edp_irp6o
{
	public:

		edp_irp6o(ui_config_entry &entry);
		~edp_irp6o();
};
ui_common_robot * robot_ontrack;
mrrocpp::lib::controller_state_t state_ontrack;
GError *error = NULL;
void *ui_synchronize_ontrack (gpointer userdata);
GtkButton* button;

#endif /* __EDP_irp6o */

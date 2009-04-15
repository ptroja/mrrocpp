
#ifndef __EDP_conveyor
#define __EDP_conveyor

#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>

#include "ui/ui_ecp_r_irp6_common.h"
#include "ui/ui_ecp_r_conveyor.h"

class edp_conveyor
{
	public:

		edp_conveyor(ui_config_entry &entry);
		~edp_conveyor();
};
ui_conveyor_robot * robot_conveyorRobot;
mrrocpp::lib::controller_state_t state_conveyorRobot;
GError *error = NULL;
void *ui_synchronize_conveyorRobot (gpointer userdata);
GtkButton* button;

#endif /* __EDP_conveyor */

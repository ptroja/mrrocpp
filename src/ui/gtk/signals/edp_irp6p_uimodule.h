
#ifndef __EDP_irp6p
#define __EDP_irp6p

#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>

#include "ui/ui_ecp_r_irp6_common.h"
#include "ui/ui_ecp_r_conveyor.h"

class edp_irp6p
{
	public:

		edp_irp6p(ui_config_entry &entry);
		~edp_irp6p();
};
ui_common_robot * robot_postument;
mrrocpp::lib::controller_state_t state_postument;
GError *error = NULL;
void *ui_synchronize_postument (gpointer userdata);
GtkButton* button;

#endif /* __EDP_irp6p */

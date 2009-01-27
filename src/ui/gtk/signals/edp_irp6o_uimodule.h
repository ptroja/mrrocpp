
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
ui_common_robot * robot;
controller_state_t state;


#endif /* __EDP_irp6o */

#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_ttt.h"
#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

ecp_task_ttt::ecp_task_ttt(configurator &_config) : common::task::ecp_task(_config) {}

void ecp_task_ttt::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);

    sg = new common::ecp_smooth_generator (*this, true, true);

    sr_ecp_msg->message("ECP loaded");
}

void ecp_task_ttt::main_task_algorithm(void)
{
	char path[666];
	int option = choose_option("1 - Euler, 2 - Joint",2);

	int pathLoaded = 0;
	while(!pathLoaded)
	{
		switch(option)
		{
			case OPTION_ONE:
				sprintf(path, "%s%s", mrrocpp_network_path, config.return_string_value("trajektoria_euler"));
				pathLoaded = 1;
			    sg->load_file_with_path(path);
				sprintf(path,"%s loaded",config.return_string_value("trajektoria"));
			    sr_ecp_msg->message("ECP Kolko-i-krzyzyk  - wcisnij start");
				break;
			case OPTION_TWO:
				sprintf(path, "%s%s", mrrocpp_network_path, config.return_string_value("trajektoria_joint"));
				pathLoaded = 1;
			    sg->load_file_with_path(path);
				sprintf(path,"%s loaded",config.return_string_value("trajektoria2"));
				break;
		}
	}

    sg->Move();
    ecp_termination_notice();
}

} // namespace irp6ot

namespace common {
namespace task {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new irp6ot::ecp_task_ttt(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp



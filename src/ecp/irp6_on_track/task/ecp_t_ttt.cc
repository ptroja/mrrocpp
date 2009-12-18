#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/task/ecp_t_ttt.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

ttt::ttt(lib::configurator &_config) : common::task::task(_config)
{
	ecp_m_robot = new robot (*this);

    sg = new common::generator::smooth (*this, true, true);
}

void ttt::main_task_algorithm(void)
{
	int option = choose_option("1 - Euler, 2 - Joint",2);

	int pathLoaded = 0;
	while(!pathLoaded)
	{
		switch(option)
		{
			case lib::OPTION_ONE:
			{
				std::string path(mrrocpp_network_path);
				path += config.value<std::string>("trajektoria_euler");

				sg->load_file_with_path(path.c_str());
			    pathLoaded = 1;

			    sr_ecp_msg->message("ECP Kolko-i-krzyzyk  - wcisnij start");
				break;
			}
			case lib::OPTION_TWO:
			{
				std::string path(mrrocpp_network_path);
				path += config.value<std::string>("trajektoria_joint");

			    sg->load_file_with_path(path.c_str());
			    pathLoaded = 1;

				break;
			}
		}
	}

    sg->Move();
    ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::ttt(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp



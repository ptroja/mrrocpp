#include <stdio.h>
#include <unistd.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_g_vis.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp/irp6_on_track/ecp_t_vis_irp6ot.h"

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


// KONSTRUKTORY
vis::vis(lib::configurator &_config) : task(_config)
{
    ecp_m_robot = new robot (*this);

    // Powolanie czujnikow
    sensor_m[lib::SENSOR_CAMERA_SA] =
        new ecp_mp::sensor::vis (lib::SENSOR_FORCE_ON_TRACK, "[vsp_section]", *this);

	// Konfiguracja wszystkich czujnikow
	BOOST_FOREACH(ecp_mp::sensor_item_t & sensor_item, sensor_m)
	{
		sensor_item.second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_item.second->configure_sensor();
	}
}


void vis::main_task_algorithm(void)
{
	generator::seven_eye_run_linear ynrlg(*this, 4);
	ynrlg.sensor_m = sensor_m;

	for(;;)
	{
		sr_ecp_msg->message("NOWA SERIA");
		ynrlg.Move();

	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::vis(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


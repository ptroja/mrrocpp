// ------------------------------------------------------------------------
//   task/ecp_t_time.cc - sledzenie konturu wersja dla dowolnego z robotow irp6
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------


#include <cstdio>
#include <cstring>
#include <map>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_g_time.h"
#include "ecp_t_time.h"
#include "ecp_mp_s_time.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


// Initilization
time::time(lib::configurator &_config) : task(_config)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (config.section_name == lib::irp6ot_m::ECP_SECTION)
		{ ecp_m_robot = new irp6ot_m::robot (*this); }
	else if (config.section_name == lib::irp6p_m::ECP_SECTION)
		{ ecp_m_robot = new irp6p_m::robot (*this); }
	else
		assert(0);
#if 0
	// Create sensors
	sensor_m[lib::SENSOR_TIME] = new ecp_mp::sensor::time (lib::SENSOR_TIME, "[vsp_time]", *this);

	// Configure sensors
	for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->configure_sensor();
	}
#endif
	tfg = new generator::time(*this, 8);
	//tfg->sensor_m = sensor_m;

	sr_ecp_msg->message("ecp time loaded");
}

void time::main_task_algorithm(void)
{
	for(;;) {
		tfg->Move();
	}
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new time(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


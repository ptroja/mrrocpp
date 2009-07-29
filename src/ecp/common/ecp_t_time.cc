// ------------------------------------------------------------------------
//   ecp_t_time.cc - sledzenie konturu wersja dla dowolnego z robotow irp6
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_g_time.h"
#include "ecp/common/ecp_t_time.h"
#include "ecp_mp/ecp_mp_s_time.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


// KONSTRUKTORY
time::time(lib::configurator &_config) : task(_config)
{
	tfg = NULL;
}

// methods for ECP template to redefine in concrete classes
void time::task_initialization(void)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
		{ ecp_m_robot = new irp6ot::ecp_irp6_on_track_robot (*this); }
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
		{ ecp_m_robot = new irp6p::ecp_irp6_postument_robot (*this); }

	// Powolanie czujnikow
	sensor_m[lib::SENSOR_TIME] = new ecp_mp::sensor::time (lib::SENSOR_TIME, "[vsp_time]", *this);

	// Konfiguracja wszystkich czujnikow
	for (ecp_mp::sensor_map::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->configure_sensor();
	}

	tfg = new generator::time(*this, 8);
	tfg->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP time loaded");
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


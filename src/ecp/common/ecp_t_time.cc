// ------------------------------------------------------------------------
//   ecp_t_time.cc - sledzenie konturu wersja dla dowolnego z robotow irp6
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_g_time.h"
#include "ecp/common/ecp_t_time.h"
#include "ecp_mp/ecp_mp_s_time.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


// KONSTRUKTORY
ecp_task_time::ecp_task_time(configurator &_config) : ecp_task(_config)
{
	tfg = NULL;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_time::task_initialization(void)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
		{ ecp_m_robot = new irp6ot::ecp_irp6_on_track_robot (*this); }
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
		{ ecp_m_robot = new irp6p::ecp_irp6_postument_robot (*this); }

	// Powolanie czujnikow
	sensor_m[SENSOR_TIME] = new ecp_mp::sensor::time (SENSOR_TIME, "[vsp_time]", *this);

	// Konfiguracja wszystkich czujnikow
	for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->configure_sensor();
	}

	tfg = new time_generator(*this, 8);
	tfg->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP time loaded");
}

void ecp_task_time::main_task_algorithm(void)
{
	for(;;) {
		tfg->Move();
	}
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_time(_config);
}
} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


/*!
 * \file ecp_t_pcbird.cc
 * \brief Class responsible for communication with pcbird (testing purposes).
 * - methods definitions.
 * \author tkornuta
 * \date 17.03.2008
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"

#include "ecp_mp/ecp_mp_s_pcbird.h"
#include "ecp/common/ecp_t_pcbird.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


/*!
 * Initialize task - robot, sensors and generators.
 */
pcbird::pcbird(lib::configurator &_config) : task(_config)
{
	// Create pcbird sensor - for testing purposes.
	sensor_m[lib::SENSOR_PCBIRD] = new ecp_mp::sensor::pcbird(lib::SENSOR_PCBIRD, "[vsp_pcbird]", *this);
	// Configure sensor.
	sensor_m[lib::SENSOR_PCBIRD]->configure_sensor();

	// Create an adequate robot. - depending on the ini section name.
	if (config.section_name == "[ecp_irp6_on_track]")
	{
		ecp_m_robot = new irp6ot::robot (*this);
		sr_ecp_msg->message("IRp6ot loaded");
	}
	else if (config.section_name == "[ecp_irp6_postument]")
	{
		ecp_m_robot = new irp6p::robot (*this);
		sr_ecp_msg->message("IRp6p loaded");
	}

	// Create generator and pass sensor to it.
	cvg = new generator::cvfradia(*this);
	cvg->sensor_m = sensor_m;
}

/*!
 * Main algorithm loop. Retrieves information from pcbird.
 */
void pcbird::main_task_algorithm(void)
{
	cvg->Move();
}

/*!
 * Returns created task object (Factory Method design pattern).
 */
task* return_created_ecp_task (lib::configurator &_config)
{
	return new pcbird(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

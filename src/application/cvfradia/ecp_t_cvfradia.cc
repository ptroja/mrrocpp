/*!
 * \file task/ecp_t_cvfradia.cc
 * \brief Class responsible for communication with cvFraDIA (testing purposes).
 * - methods definitions.
 * \author tkornuta
 * \date 17.03.2008
 */

#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "base/ecp_mp/sensor/ecp_mp_s_cvfradia.h"
#include "ecp_t_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

/*!
 * Initialize task - robot, sensors and generators.
 */
cvfradia::cvfradia(lib::configurator &_config) : task(_config)
{
	// Create cvFraDIA sensor - for testing purposes.
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = new ecp_mp::sensor::cvfradia(ecp_mp::sensor::SENSOR_FRADIA, "[vsp_cvfradia]", *this, sizeof(lib::sensor_image_t::sensor_union_t::fradia_t));
	// Configure sensor.
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA]->configure_sensor();

	// Create an adequate robot. - depending on the ini section name.
    if (config.section_name == ECP_SECTION)
    {
        ecp_m_robot = new irp6ot_m::robot (*this);
        sr_ecp_msg->message("IRp6ot loaded");
    }
    else if (config.section_name == lib::irp6p_m::ECP_SECTION)
    {
        ecp_m_robot = new irp6p_m::robot (*this);
        sr_ecp_msg->message("IRp6p loaded");
    }

    // Create generator and pass sensor to it.
	cvg = new generator::cvfradia(*this);
	cvg->sensor_m = sensor_m;
}


/*!
 * Main algorithm loop. Retrieves information from cvFraDIA.
 */
void cvfradia::main_task_algorithm(void)
{
	cvg->Move();
}

/*!
 * Returns created task object (Factory Method design pattern).
 */
task* return_created_ecp_task (lib::configurator &_config)
{
	return new cvfradia(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

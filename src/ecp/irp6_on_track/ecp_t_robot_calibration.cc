/*!
 /* file ecp_t_robot_calibration.cc
 * Methods responsible for moving robot for robot calibration task with pcbird
 * \author manibaktha
 * \date 07.05.2009
 */

#include "ecp/irp6_on_track/ecp_t_robot_calibration.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//ECP Task constructor
robot_calibration::robot_calibration(lib::configurator &_config): task(_config)
{
	sr_ecp_msg->message("(TASK: initialization");

	// Create an adequate robot. - depending on the ini section name.
/*	if (strcmp(config.section_name, ECP_IRP6_ON_TRACK_SECTION) == 0)
	{
    		ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    		sr_ecp_msg->message("IRp6 on Track loaded");
	}*/

	// Create calibration generator.
	generator = new generator::robotcalibgen(*this);
	// Create pcbird sensor in the generato sensors list.
 	generator->sensor_m[lib::SENSOR_PCBIRD] = new ecp_mp::sensor::pcbird("[vsp_calibration_pcbird]", *this);
   	sr_ecp_msg->message("TASK: PCBIRD sensor added to list");
}

void robot_calibration::main_task_algorithm(void)
{
	// Execute generator Move method.
	generator->Move();
	// Send termination notice to the MP process.
	ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	printf("Returning task object\n");
	return new irp6ot::task::robot_calibration(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


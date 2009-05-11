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
//	sr_ecp_msg->message("ecp task constructor");
}

robot_calibration::~robot_calibration()
{

}

// methods for ECP template to redefine in concrete classes
void robot_calibration::task_initialization(void)
{
	sr_ecp_msg->message("task_initialization");

	// create pcbird sensor - for testing purposes.
	sensor_m[lib::SENSOR_PCBIRD] = new ecp_mp::sensor::pcbird(lib::SENSOR_PCBIRD, "[vsp_calibration_pcbird]", *this);
	sensor_m[lib::SENSOR_PCBIRD]->configure_sensor();
   	sr_ecp_msg->message("PCBIRD sensor loaded");

/*	// Create an adequate robot. - depending on the ini section name.
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
    		ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    		sr_ecp_msg->message("IRp6 on Track loaded");
	}
	*/
	// Create spots generator and pass sensor to it.
	generator = new generator::robotcalibgen(*this);
 	//generator->sensor_m = sensor_m;
 	
}

void robot_calibration::main_task_algorithm(void)
{

	sr_ecp_msg->message("robot_calibration::main_task_algorithm");
	for(int i=0; i<100; i++)
		generator->Move();
	
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


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp_t_graspit.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
graspit::graspit(lib::configurator &_config): task(_config)
{
    if (config.section_name == ECP_IRP6_ON_TRACK_SECTION)
    {
        ecp_m_robot = new irp6ot::robot (*this);
    }
    else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION)
    {
        ecp_m_robot = new irp6p::robot (*this);
    }

	smoothgen2 = new common::generator::smooth(*this, true);
	smoothgen2->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP loaded graspit");
};

void graspit::main_task_algorithm(void ) {

	sr_ecp_msg->message("ECP graspit ready");

	ecp_termination_notice();

};

} // namespace task
} // namespace common

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new common::task::graspit(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp



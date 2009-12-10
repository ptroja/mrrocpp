#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/irp6_postument/ecp_t_kcz_force.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

//Constructors
kcz_force::kcz_force(lib::configurator &_config): task(_config)
{
	ecp_m_robot = new robot(*this);

	//delay(20000);
	nose_run = new common::generator::tff_nose_run(*this, 8);
	nose_run->configure_pulse_check (true);

	sr_ecp_msg->message("ECP loaded kcz_force");
};

void kcz_force::main_task_algorithm(void ) {
	//ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	//smoothgen2 = new ecp_smooth2_generator(*this, true);
	//sr_ecp_msg->message("ECP loaded smooth2_test");

	sr_ecp_msg->message("ECP kcz_force ready");

	nose_run->Move();

	//printf("wielkosc listy: %d\n", smoothgen2->pose_list_length());
	//fflush();

	ecp_termination_notice();
};

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6p::task::kcz_force(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp



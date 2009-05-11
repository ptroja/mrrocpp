#include <stdio.h>

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_pw_kolo_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Konstruktory
pw::pw(lib::configurator &_config) : task(_config)
{}

void pw::task_initialization(void)
{
	ecp_m_robot= new ecp_irp6_on_track_robot(*this);
	kolo_gen= new generator::pw_kolo(*this);

	sr_ecp_msg->message("ECP PW loaded");
}


void pw::main_task_algorithm(void)
{
	sr_ecp_msg->message("Przed MOVE");
	kolo_gen->Move();
	sr_ecp_msg->message("Po MOVE");

	sr_ecp_msg->message("przed wait_for_stop\n");
	ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot::task::pw(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#include <stdio.h>

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/task/ecp_t_pw_kolo_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Konstruktory
pw::pw(lib::configurator &_config) : task(_config)
{
	ecp_m_robot= new robot(*this);
	kolo_gen= new generator::pw_kolo(*this);
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


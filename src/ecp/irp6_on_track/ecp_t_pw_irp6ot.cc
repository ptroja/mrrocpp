#include <stdio.h>

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_pw_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace common {

//Konstruktory
ecp_task_pw_irp6ot::ecp_task_pw_irp6ot(configurator &_config) : ecp_task(_config)
{}

void ecp_task_pw_irp6ot::task_initialization(void)
{
	ecp_m_robot=new ecp_irp6_on_track_robot(*this);
	kolo_gen=new ecp_g_pw_kolo(*this);

	sr_ecp_msg->message("ECP PW loaded");
}


void ecp_task_pw_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("Przed MOVE");
	kolo_gen->Move();
	sr_ecp_msg->message("Po MOVE");

	sr_ecp_msg->message("przed wait_for_stop\n");
	ecp_termination_notice();
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_task_pw_irp6ot(_config);
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp


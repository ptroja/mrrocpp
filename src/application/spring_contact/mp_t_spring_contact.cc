/*!
 * @file
 * @brief File contains mp_task class definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */
#include "mp_t_spring_contact.h"

// ecp generators to be commanded
#include "ecp_mp_g_spring_contact.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/tff_nose_run/ecp_mp_g_tff_nose_run.h"

// mp_robots headers
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"

namespace mrrocpp {
namespace mp {
namespace task {

spring_contact::spring_contact(lib::configurator &_config) :
		task(_config)
{
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void spring_contact::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
}

void spring_contact::main_task_algorithm(void)
{

	sr_ecp_msg->message("New spring_contact series");

	// sekwencja generator na wybranym manipulatorze
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, (int) 0, "", lib::irp6p_m::ROBOT_NAME);

	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	for (;;) {

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) 0, "", lib::irp6p_m::ROBOT_NAME);

		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		set_next_ecp_state(ecp_mp::generator::SPRING_CONTACT, (int) 0, "", lib::irp6p_m::ROBOT_NAME);

		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
	}

	sr_ecp_msg->message("END");
}

task* return_created_mp_task(lib::configurator &_config)
{
	return new spring_contact(_config);
}

} // namespace task
} // namespace mp
} // namespace mrrocpp

/*
 * mp_t_neuron.cpp
 *
 *  Created on: Jun 25, 2010
 *      Author: tbem
 */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "robot/irp6ot_m/irp6ot_m_const.h"

#include "base/mp/mp_task.h"

#include "mp_t_neuron.h"
#include "ecp_mp_t_neuron.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new neuron(_config);
}

neuron::neuron(lib::configurator &_config) :
	task(_config)
{
}

void neuron::main_task_algorithm(void)
{
	sr_ecp_msg->message("Neuron task initialization");

	set_next_ecps_state(ecp_mp::task::ECP_T_NEURON, (int) 5, "", 0, 1, lib::ROBOT_IRP6OT_M.c_str());
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_IRP6OT_M.c_str());

	sr_ecp_msg->message("END");
}

neuron::~neuron()
{
	// TODO Auto-generated destructor stub
}

} //task
} //mp
} //mrrocpp

/*
 * mp_t_neuron.cpp
 *
 *  Created on: Jun 25, 2010
 *      Author: tbem
 */

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/mp/mp_task.h"

#include "mp_t_neuron.h"
#include "ecp_mp_t_neuron.h"


#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

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

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void neuron::create_robots()
{
	//ACTIVATE_MP_ROBOT(conveyor);
	//ACTIVATE_MP_ROBOT(speaker);
	//ACTIVATE_MP_ROBOT(irp6m);
	//ACTIVATE_MP_ROBOT(polycrank);
	//ACTIVATE_MP_ROBOT(bird_hand);
	//ACTIVATE_MP_ROBOT(spkm);
	//ACTIVATE_MP_ROBOT(smb);
	//ACTIVATE_MP_ROBOT(shead);
	//ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	//ACTIVATE_MP_ROBOT(irp6p_tfg);
	//ACTIVATE_MP_ROBOT(irp6p_m);
	//ACTIVATE_MP_ROBOT(sarkofag);

	//ACTIVATE_MP_DEFAULT_ROBOT(electron);
	//ACTIVATE_MP_DEFAULT_ROBOT(speechrecognition);
	//ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

void neuron::main_task_algorithm(void)
{
	sr_ecp_msg->message("Neuron task initialization");

	set_next_ecps_state(ecp_mp::task::ECP_T_NEURON, (int) 5, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	run_extended_empty_gen_and_wait(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());

	sr_ecp_msg->message("END");
}

neuron::~neuron()
{
	// TODO Auto-generated destructor stub
}

} //task
} //mp
} //mrrocpp

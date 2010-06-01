// ------------------------------------------------------------------------
//   task/ecp_t_sk.cc - sledzenie konturu wersja dla dowolnego z robotow irp6
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"


#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "application/sk/ecp_g_sk.h"

#include "application/sk/ecp_t_sk.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


// KONSTRUKTORY
sk::sk(lib::configurator &_config) : task(_config)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (config.section_name == ECP_IRP6OT_M_SECTION)
	{
		ecp_m_robot = new irp6ot_m::robot (*this);
	}
	else if (config.section_name == ECP_IRP6P_M_SECTION)
	{
		ecp_m_robot = new irp6p_m::robot (*this);
	} else {
		// TODO: throw
	}

	nrg = new generator::tff_nose_run(*this, 8);

	nrg->configure_pulse_check (true);

	yefg = new generator::y_edge_follow_force (*this, 8);
	befg = new generator::bias_edp_force(*this);

	switch (ecp_m_robot->robot_name)
	{
	case lib::ROBOT_IRP6OT_M:
		sr_ecp_msg->message("ECP sk irp6ot loaded");
		break;
	case lib::ROBOT_IRP6P_M:
		sr_ecp_msg->message("ECP sk irp6p loaded");
		break;
	default:
		fprintf(stderr, "%s:%d unknown robot type\n", __FILE__, __LINE__);
	}

	// sprawdzenie dodatkowej opcji w konfiguracji dotyczacej uruchomienie zapamietywania trajektorii do pliku
	if (config.exists("save_activated"))
	{
		save_activated = (bool) config.value<int>("save_activated");
	}
	else
	{
		save_activated = false;
	}
}


void sk::main_task_algorithm(void)
{
	//   weight_meassure_generator wmg(*this, 0.3, 2);

	for(;;)
	{
		sr_ecp_msg->message("NOWA SERIA");
		sr_ecp_msg->message("FORCE SENSOR BIAS");
		befg->Move();
		sr_ecp_msg->message("Wodzenie do pozycji sledzenia konturu");
		sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
		nrg->Move();
		//			sr_ecp_msg->message("wazenie");
		//			wmg.Move();

		// usuniecie listy o ile istnieje
		yefg->flush_pose_list();

		sr_ecp_msg->message("Sledzenie konturu");
		sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
		yefg->Move();
		if ( save_activated && operator_reaction ("Save drawing ") )
		{
			sr_ecp_msg->message("Zapisywanie trajektorii");
			yefg->save_file (lib::ECP_PF_VELOCITY);
		}

	}
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new sk(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


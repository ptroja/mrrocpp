// ------------------------------------------------------------------------
//   ecp_t_sk.cc - sledzenie konturu wersja dla dowolnego z robotow irp6
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
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp/common/ecp_t_sk.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


// KONSTRUKTORY
sk::sk(lib::configurator &_config) : base(_config)
{
	nrg = NULL;
	yefg = NULL;
}

// methods for ECP template to redefine in concrete classes
void sk::task_initialization(void)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
		ecp_m_robot = new irp6ot::ecp_irp6_on_track_robot (*this);
	}
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
	{
		ecp_m_robot = new irp6p::ecp_irp6_postument_robot (*this);
	}

	usleep(1000*100);

	nrg = new generator::tff_nose_run(*this, 8);


	nrg->configure_pulse_check (true);

	yefg = new generator::y_edge_follow_force (*this, 8);
	befg = new generator::bias_edp_force(*this);


	switch (ecp_m_robot->robot_name)
	{
	case ROBOT_IRP6_ON_TRACK:
		sr_ecp_msg->message("ECP sk irp6ot loaded");
		break;
	case ROBOT_IRP6_POSTUMENT:
		sr_ecp_msg->message("ECP sk irp6p loaded");
		break;
	default:
		fprintf(stderr, "%s:%d unknown robot type\n", __FILE__, __LINE__);
	}

	// sprawdzenie dodatkowej opcji w konfiguracji dotyczacej uruchomienie zapamietywania trajektorii do pliku
	if (config.exists("save_activated"))
	{
		save_activated = (bool) config.return_int_value("save_activated");
	}
	else
	{
		save_activated = false;
	}

};


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
			yefg->save_file (PF_VELOCITY);
		}

	}
}

base* return_created_ecp_task (lib::configurator &_config)
{
	return new sk(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


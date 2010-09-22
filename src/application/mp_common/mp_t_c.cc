/*!
 * @file
 * @brief File contains mp common task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup
 */

#include <cstdio>
#include <unistd.h>
#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"

#include "base/mp/MP_main_error.h"
#include "mp_t_c.h"
#include "base/mp/generator/mp_g_extended_empty.h"

#include <boost/foreach.hpp>

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6m/mp_r_irp6m.h"
#include "robot/speaker/mp_r_speaker.h"
#include "robot/polycrank/mp_r_polycrank.h"
#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/shead/mp_r_shead.h"
#include "robot/spkm/mp_r_spkm.h"
#include "robot/smb/mp_r_smb.h"
#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"
#include "robot/player/const_player.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new cxx(_config);
}

cxx::cxx(lib::configurator &_config) :
	task(_config)
{
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void cxx::create_robots()
{
	ACTIVATE_MP_ROBOT(conveyor);
	ACTIVATE_MP_ROBOT(speaker);
	ACTIVATE_MP_ROBOT(irp6m);
	ACTIVATE_MP_ROBOT(polycrank);
	ACTIVATE_MP_ROBOT(bird_hand);
	ACTIVATE_MP_ROBOT(spkm);
	ACTIVATE_MP_ROBOT(smb);
	ACTIVATE_MP_ROBOT(shead);
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(sarkofag);

	ACTIVATE_MP_DEFAULT_ROBOT(electron);
	ACTIVATE_MP_DEFAULT_ROBOT(speechrecognition);
	ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

void cxx::main_task_algorithm(void)
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->ecp_reply_package.reply = lib::ECP_ACKNOWLEDGE;

				}

	generator::extended_empty empty_gen(*this); // "Pusty" generator
	empty_gen.robot_m = robot_m;

	// Zlecenie wykonania kolejnego makrokroku
	empty_gen.Move();
}

} // namespace task
} // namespace mp
} // namespace mrrocpp



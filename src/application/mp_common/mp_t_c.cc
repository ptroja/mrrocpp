/*!
 * @file
 * @brief File contains mp common task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup
 */

#include <cstdio>
#include <unistd.h>

#include <boost/foreach.hpp>

#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"

#include "base/mp/MP_main_error.h"
#include "mp_t_c.h"
#include "base/mp/generator/mp_g_wait_for_task_termination.h"

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "robot/polycrank/mp_r_polycrank.h"
#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/shead/mp_r_shead1.h"
#include "robot/shead/mp_r_shead2.h"
#include "robot/spkm/mp_r_spkm1.h"
#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb1.h"
#include "robot/smb/mp_r_smb2.h"
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

	ACTIVATE_MP_ROBOT(polycrank);
	ACTIVATE_MP_ROBOT(bird_hand);
	ACTIVATE_MP_ROBOT(spkm1);
	ACTIVATE_MP_ROBOT(spkm2);
	ACTIVATE_MP_ROBOT(smb1);
	ACTIVATE_MP_ROBOT(smb2);
	ACTIVATE_MP_ROBOT(shead1);
	ACTIVATE_MP_ROBOT(shead2);
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(sarkofag);

}

void cxx::main_task_algorithm(void)
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					// Drop the 'const' qualifier
					lib::ECP_REPLY_PACKAGE & ecp_reply_package =
							const_cast <lib::ECP_REPLY_PACKAGE&> (robot_node.second->ecp_reply_package);

					// NOTE: this does write to the INPUT buffer (?!)
					// Probably this should be solved with a proper initialization.
					ecp_reply_package.reply = lib::ECP_ACKNOWLEDGE;
				}

	generator::wait_for_task_termination wtf_gen(*this, false); // "Pusty" generator
	wtf_gen.robot_m = robot_m;

	// Zlecenie wykonania kolejnego makrokroku
	wtf_gen.Move();

}

} // namespace task
} // namespace mp
} // namespace mrrocpp



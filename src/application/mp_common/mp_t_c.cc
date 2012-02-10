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

#include "mp_t_c.h"
#include "base/mp/generator/mp_g_wait_for_task_termination.h"

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"


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


#if (R_BIRD_HAND == 1)
	ACTIVATE_MP_
	ROBOT(bird_hand);
#endif
#if (R_012 == 1)
	ACTIVATE_MP_ROBOT(conveyor);
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(sarkofag);
#endif

}

void cxx::main_task_algorithm(void)
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				// Drop the 'const' qualifier
				lib::ECP_REPLY_PACKAGE & ecp_reply_package =
						const_cast <lib::ECP_REPLY_PACKAGE&>(robot_node.second->ecp_reply_package);

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


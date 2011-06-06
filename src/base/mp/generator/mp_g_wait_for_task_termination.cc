/*!
 * @file
 * @brief File contains mp empty generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstring>

#include <boost/foreach.hpp>

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_robot.h"

#include "robot/player/ecp_mp_t_player.h"
#include "base/mp/generator/mp_g_wait_for_task_termination.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// ###############################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################

wait_for_task_termination::wait_for_task_termination(task::task& _mp_task, bool _check_task_termination_in_first_step =
		true) :
	generator(_mp_task), check_task_termination_in_first_step(true)
{
	check_task_termination_in_first_step = _check_task_termination_in_first_step;
	wait_for_ECP_pulse = true;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool wait_for_task_termination::first_step()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->communicate_with_ecp = false;
				}

	if (check_task_termination_in_first_step) {
		// usuwamy te roboty, ktore juz odpoweidzialy
		BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED) {
							robot_m.erase(robot_node.first);
						}
					}

		if (robot_m.empty()) {
			return false;
		} else {
			return true;
		}
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool wait_for_task_termination::next_step()
{

	// usuwamy te roboty, ktore juz odpoweidzialy
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED) {
						robot_m.erase(robot_node.first);
					}
				}

	if (robot_m.empty()) {

		return false;
	} else {

		return true;
	}
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


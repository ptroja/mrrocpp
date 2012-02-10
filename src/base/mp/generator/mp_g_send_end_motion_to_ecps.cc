/*!
 * @file
 * @brief File contains mp send_end_motion_to_ecps generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <boost/foreach.hpp>

#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"

#include "base/mp/generator/mp_g_send_end_motion_to_ecps.h"

namespace mrrocpp {
namespace mp {
namespace generator {

send_end_motion_to_ecps::send_end_motion_to_ecps(task::task& _mp_task) :
	generator(_mp_task)
{
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool send_end_motion_to_ecps::first_step()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
	{
		robot_node.second->mp_command.command = lib::END_MOTION;
		robot_node.second->communicate_with_ecp = true;
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool send_end_motion_to_ecps::next_step()
{
	return false;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp

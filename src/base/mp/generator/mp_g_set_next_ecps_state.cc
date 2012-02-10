/*!
 * @file
 * @brief File contains mp set_next_ecps_state generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstring>

#include <boost/foreach.hpp>

#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"

#include "base/mp/generator/mp_g_set_next_ecps_state.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// generator for setting the next ecps state

set_next_ecps_state::set_next_ecps_state(task::task& _mp_task) :
		generator(_mp_task)
{
	wait_for_ECP_message = true;
}


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool set_next_ecps_state::first_step()
{
	robots_to_reply = robot_m;

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->mp_command.command = lib::NEXT_STATE;
				robot_node.second->mp_command.ecp_next_state = ecp_next_state;
				robot_node.second->communicate_with_ecp = true;
			}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool set_next_ecps_state::next_step()
{
//	sr_ecp_msg.message(lib::NON_FATAL_ERROR, "set_next_ecps_state::next_step");

// tutuaj oczekujemy aż wszystkie roboty potwierdzą otrzymanie rozkazu przez ecp_acknowledge
// korzystamy ze zbioru robot_m
// najpierw wylaczamy wysylanie czegokolwiek do robotow
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->communicate_with_ecp = false;
			}

	// usuwamy te roboty, ktore juz odpoweidzialy
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_reply)
			{
				if (robot_node.second->reply.isFresh()) {
					if (robot_node.second->ecp_reply_package.reply != lib::ECP_ACKNOWLEDGE) {
						std::stringstream temp_message;
						temp_message << "set_next_ecps_state != lib::ECP_ACKNOWLEDGE robot ("
								<< robot_node.second->robot_name << ")" << std::endl;
						sr_ecp_msg.message(lib::NON_FATAL_ERROR, temp_message.str());
					}
					robots_to_reply.erase(robot_node.first);
				}
			}

	return (robots_to_reply.empty() ? false : true);
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


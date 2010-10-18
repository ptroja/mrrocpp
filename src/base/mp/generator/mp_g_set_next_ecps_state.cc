/*!
 * @file
 * @brief File contains mp set_next_ecps_state generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstring>

#include <boost/foreach.hpp>

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_robot.h"

#include "robot/player/ecp_mp_t_player.h"
#include "base/mp/generator/mp_g_set_next_ecps_state.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// generator for setting the next ecps state

set_next_ecps_state::set_next_ecps_state(task::task& _mp_task) :
	generator(_mp_task)
{
}

void set_next_ecps_state::configure(std::string l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant, const char* l_mp_2_ecp_next_state_string, int str_len)
{
	strcpy(ecp_next_state.mp_2_ecp_next_state, l_mp_2_ecp_next_state.c_str());
	ecp_next_state.mp_2_ecp_next_state_variant = l_mp_2_ecp_next_state_variant;
	if (l_mp_2_ecp_next_state_string) {
		if (str_len == 0) {
			strcpy(ecp_next_state.mp_2_ecp_next_state_string, l_mp_2_ecp_next_state_string);
		} else {
			memcpy(ecp_next_state.mp_2_ecp_next_state_string, l_mp_2_ecp_next_state_string, str_len);
		}
	}
}

void set_next_ecps_state::configure(const lib::playerpos_goal_t &_goal)
{
	strcpy(ecp_next_state.mp_2_ecp_next_state, ecp_mp::task::ECP_GEN_PLAYERPOS.c_str());
	ecp_next_state.playerpos_goal = _goal;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool set_next_ecps_state::first_step()
{
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
	return false;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


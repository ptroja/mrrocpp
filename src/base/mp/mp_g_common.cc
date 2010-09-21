/*!
 * @file
 * @brief File contains mp common generators definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <cstring>

#include <boost/foreach.hpp>

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_robot.h"

#include "robot/player/ecp_mp_t_player.h"
#include "base/mp/mp_g_common.h"

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

// ###############################################################
// Rozszerzony generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################

extended_empty::extended_empty(task::task& _mp_task) :
	generator(_mp_task)
{
	activate_trigger = true;
}

void extended_empty::configure(bool l_activate_trigger)
{
	activate_trigger = l_activate_trigger;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool extended_empty::first_step()
{
	wait_for_ECP_pulse = true;
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->mp_command.command = lib::NEXT_POSE;
					robot_node.second->mp_command.instruction.instruction_type = lib::QUERY;
					robot_node.second->communicate_with_ecp = false;
				}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool extended_empty::next_step()
{
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// Na podstawie ecp_reply dla poszczegolnych robotow nalezy okreslic czy
	// skonczono zadanie uzytkownika

	// 	if (trigger) printf("Yh\n"); else printf("N\n");
	// printf("mp next step\n");
	// UWAGA: dzialamy na jednoelementowej liscie robotow

	if (check_and_null_trigger() && activate_trigger) {
		return false;
	}

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->communicate_with_ecp = (robot_node.second->new_pulse);
				}

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED) {
						//  sr_ecp_msg.message("w mp task terminated");
						return false;
					}
				}

	return true;
}

// ###############################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################

empty::empty(task::task& _mp_task) :
	generator(_mp_task)
{
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool empty::first_step()
{
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// Inicjacja generatora trajektorii
	// printf("mp first step\n");
	// wait_for_ECP_pulse = true;
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->mp_command.command = lib::NEXT_POSE;
					robot_node.second->mp_command.instruction.instruction_type = lib::QUERY;
					robot_node.second->communicate_with_ecp = true;
				}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool empty::next_step()
{
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// Na podstawie ecp_reply dla poszczegolnych robotow nalezy okreslic czy
	// skonczono zadanie uzytkownika

	// obrazu danych wykorzystywanych przez generator

	// 	if (trigger) printf("Yh\n"); else printf("N\n");
	// printf("mp next step\n");
	// UWAGA: dzialamy na jednoelementowej liscie robotow

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED) {
						sr_ecp_msg.message("w mp task terminated");
						return false;
					}
				}

	return true;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


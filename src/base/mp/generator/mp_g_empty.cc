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
#include "base/mp/generator/mp_g_empty.h"

namespace mrrocpp {
namespace mp {
namespace generator {

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


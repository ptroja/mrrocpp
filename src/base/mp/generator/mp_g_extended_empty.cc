/*!
 * @file
 * @brief File contains mp extended_empty generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <boost/foreach.hpp>

#include "base/mp/mp_robot.h"

#include "base/mp/generator/mp_g_extended_empty.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// ###############################################################
// Rozszerzony generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################

extended_empty::extended_empty(task::task& _mp_task) :
	generator(_mp_task), activate_trigger(true)
{
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

	// najpierw budujemy tymczasowa liste robots_to_reply

	//	potem sprawdzamy, ktore z robotow sa juz w task_terminated

	// potem o ikle ktores jeszcze pozostaly czekamy na kolejny wiasdomosc od ecp


	// w przeciwny razie odbieramy klejene wiadonosci w next_state


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
					if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED) {
						//  sr_ecp_msg.message("w mp task terminated");
						return false;
					}
				}

	return true;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp

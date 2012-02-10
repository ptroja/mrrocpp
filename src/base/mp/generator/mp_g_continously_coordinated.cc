/*!
 * @file
 * @brief File contains mp empty generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <boost/foreach.hpp>

#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"

#include "base/mp/generator/mp_g_continously_coordinated.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// ###############################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################

continously_coordinated::continously_coordinated(task::task& _mp_task) :
	generator(_mp_task), cycle_counter(0)
{
	wait_for_ECP_message = true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool continously_coordinated::next_step()
{
	// cykl oznacza odebranie danych od wszystkich robotow


	// wszystkim ustawiamy rozkaz na next_pose
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{

					robot_node.second->mp_command.command = lib::NEXT_POSE;
				}

	// wpisujemy liste robotow do odpowiedzi o ile byla pusta
	if (robots_to_reply.empty()) {
		robots_to_reply = robot_m;
	}

	// usuwamy te roboty, ktore juz odpoweidzialy
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_reply)
				{
					if (robot_node.second->reply.isFresh()) {

						robots_to_reply.erase(robot_node.first);

					}
				}

	// jezeli wszystkie odpowiedzialy to mozemy odaplic wlasciwy kod generatora trajetktorii
	if (robots_to_reply.empty()) {
		cycle_counter++;
		// wlacz komunikacje ze wszystkimi robotami
		BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
					{

						robot_node.second->communicate_with_ecp = true;

					}

		return next_step_inside();
		// w przeciwnym wypadku wylaczamy komuniakacje (w sensie wysylania rozkazu wiadomosci)
	} else {
		// wylacz komunikacje ze wszystkimi robotami
		BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
					{

						robot_node.second->communicate_with_ecp = false;

					}
	}
	return true;

}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


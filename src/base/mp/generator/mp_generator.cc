/*!
 * @file
 * @brief File contains mp base generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_generator.h"
#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace mp {
namespace generator {

generator::generator(task::task_base & _mp_task) :
	ecp_mp::generator::generator(*_mp_task.sr_ecp_msg), mp_t(_mp_task), wait_for_ECP_message(false)
{
}

// ---------------------------------------------------------------
void generator::Move()
{
	// Set the utility counter to zero
	node_counter = 0;

	// (Inicjacja) generacja pierwszego kroku ruchu
	if (!first_step())
		return;

	do { // realizacja ruchu
		// zadanie przygotowania danych od czujnikow
		initiate_sensors_readings();

		// wykonanie kroku ruchu przez wybrane roboty (z flaga 'communicate_with_ecp')
		execute_all();

		// odczytanie danych z wszystkich czujnikow
		get_sensors_readings();

		// oczekiwanie na puls z ECP lub UI
		mp_t.receive_ui_or_ecp_message(*this);

		node_counter++;
	} while (next_step());

	// kasujemy znacznik swiezosci buforow
	BOOST_FOREACH(const common::robot_pair_t & robot_node, mp_t.robot_m)
	{
		if (robot_node.second->reply.isFresh()) {
			robot_node.second->reply.markAsUsed();
		}
	}
}
// ------------------------------------------------------------------------

void generator::execute_all()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				if (robot_node.second->communicate_with_ecp) {
					robot_node.second->execute_motion();
				}
			}
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


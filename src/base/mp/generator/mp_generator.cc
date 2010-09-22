/*!
 * @file
 * @brief File contains mp base generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_g_empty.h"
#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace mp {
namespace generator {

generator::generator(task::task& _mp_task) :
	ecp_mp::generator::generator(*_mp_task.sr_ecp_msg), mp_t(_mp_task), wait_for_ECP_pulse(false)
{
}

// ---------------------------------------------------------------
void generator::Move()
{
	// Funkcja zwraca false gdy samoistny koniec ruchu
	// Funkcja zwraca true gdy koniec ruchu wywolany jest przez STOP

	// czyszczenie aby nie czekac na pulsy z ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, mp_t.robot_m)
				{
					if (robot_node.second->new_pulse) {
						robot_node.second->new_pulse_checked = false;
					}
				}

	// by Y - linia ponizej dodana 26.02.2007 - usunac komentarz jak bedzie dzialalo
	// ze wzgledu na obluge pulsow z UI w szczegolnosci stopu i wstrzymania
	mp_t.mp_receive_ui_or_ecp_pulse(mp_t.robot_m, *this);

	// czyszczenie aby nie czekac na pulsy z ECP
	BOOST_FOREACH(const common::robot_pair_t & robot_node, mp_t.robot_m)
				{
					if (robot_node.second->new_pulse) {
						robot_node.second->new_pulse_checked = false;
					}
				}

	node_counter = 0;
	// (Inicjacja) generacja pierwszego kroku ruchu
	if (!first_step())
		return;

	do { // realizacja ruchu

		// zadanie przygotowania danych od czujnikow
		mp_t.all_sensors_initiate_reading(sensor_m);

		// wykonanie kroku ruchu przez wybrane roboty (z flaga 'communicate_with_ecp')
		mp_t.execute_all(robot_m);

		// odczytanie danych z wszystkich czujnikow
		mp_t.all_sensors_get_reading(sensor_m);

		// oczekiwanie na puls z ECP lub UI
		mp_t.mp_receive_ui_or_ecp_pulse(mp_t.robot_m, *this);

		node_counter++;
	} while (next_step());
}
// ------------------------------------------------------------------------


MP_error::MP_error(lib::error_class_t err0, uint64_t err1) :
	error_class(err0), error_no(err1)
{
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


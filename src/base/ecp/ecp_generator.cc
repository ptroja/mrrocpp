/*!
 * @file
 * @brief File contains ecp base generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

generator::generator(common::task::task& _ecp_task) :
	ecp_mp::generator::generator(*(ecp_t.sr_ecp_msg)), ecp_t(_ecp_task), communicate_with_mp_in_move(true),
			the_robot(ecp_t.ecp_m_robot)
{
}

generator::~generator()
{
}

bool generator::is_EDP_error(robot::ecp_robot& _robot) const
{
	// Sprawdzenie czy nie wystapil blad w EDP
	// Funkcja zaklada, ze error_no zostalo zaktualizowane
	// za pomoca conveyor_generator::get_reply
	if (_robot.reply_package.error_no.error0 || _robot.reply_package.error_no.error1) {
		return true;
	} else {
		return false;
	}
}

void generator::move_init()
{

	// domyslnie komunikumemy sie z robotem o ile on jest
	if (the_robot) {
		// default communication mode
		the_robot->communicate_with_edp = true;
		// clear data ports in case there is old data in it;
		the_robot->port_manager.clear_data_ports();

	}
	// domyslny tryb koordynacji
	ecp_t.continuous_coordination = false;

	// generacja pierwszego kroku ruchu
	node_counter = 0;
	ecp_t.set_ecp_reply(lib::ECP_ACKNOWLEDGE);

}

void generator::Move()
{
	// Funkcja ruchu dla ECP

	move_init();

	if (!first_step() || (communicate_with_mp_in_move && !ecp_t.mp_buffer_receive_and_send())) {
		return; // Warunek koncowy spelniony w pierwszym kroku
	}

	do { // realizacja ruchu

		// zadanie przygotowania danych od czujnikow
		ecp_t.all_sensors_initiate_reading(sensor_m);

		if (the_robot) {

			// zlecenie ruchu SET oraz odczyt stanu robota GET
			if (!(ecp_t.continuous_coordination)) {
				the_robot->create_command();
			}

			// wykonanie kroku ruchu
			if (the_robot->communicate_with_edp) {

				execute_motion();

				the_robot->get_reply();
			}
		}

		// odczytanie danych z wszystkich czujnikow
		ecp_t.all_sensors_get_reading(sensor_m);

		node_counter++;
		if (ecp_t.pulse_check()) {
			trigger = true;
		}

	} while (next_step() && (!communicate_with_mp_in_move || ecp_t.mp_buffer_receive_and_send()));
}

void generator::execute_motion(void)
{
	the_robot->execute_motion();
}

ECP_error::ECP_error(lib::error_class_t err_cl, uint64_t err_no, uint64_t err0, uint64_t err1) :
	error_class(err_cl), error_no(err_no)
{
	error.error0 = err0;
	error.error1 = err1;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp



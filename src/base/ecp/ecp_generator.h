#if !defined(_ECP_GENERATOR_H)
#define _ECP_GENERATOR_H

/*!
 * @file
 * @brief File contains ecp base generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <boost/shared_ptr.hpp>

#include "base/ecp_mp/ecp_mp_generator.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief Base class of all ecp generators
 * The generator both generates command and checks terminal condition
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
template <typename ECP_ROBOT_T>
class _generator : public ecp_mp::generator::generator
{
protected:
	/**
	 * @brief ECP task object type
	 */
	typedef ECP_ROBOT_T robot_t;

	/**
	 * @brief ECP task object type
	 */
	typedef common::task::_task <ECP_ROBOT_T> task_t;

	/**
	 * @brief ECP generator itself object type
	 */
	typedef _generator <ECP_ROBOT_T> generator_t;

	/**
	 * @brief ECP task object reference
	 */
	task_t & ecp_t;

public:
	/**
	 * @brief Main generator method to execute transition cycle
	 */
	void Move(void)
	{
		// Funkcja ruchu dla ECP

		move_init();

		if (!first_step()) {
			return; // Warunek koncowy spelniony w pierwszym kroku
		}

		// realizacja ruchu
		do {
			if (ecp_t.peek_mp_message()) {
				// END_MOTION received
				break;
			}

			// zlecenie przygotowania danych przez czujniki
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

		} while (next_step());
		ecp_t.command.markAsUsed();
	}

	/**
	 * @brief communicates with EDP
	 */
	virtual void execute_motion(void)
	{
		the_robot->execute_motion();
	}

	/**
	 * @brief initiates Move method
	 */
	void move_init(void)
	{
		// domyslnie komunikujemy sie z robotem o ile on jest
		if (the_robot) {
			// default communication mode
			the_robot->communicate_with_edp = true;
			// clear data ports in case there is old data there;
			the_robot->port_manager.clear_data_ports();
		}
		// domyslny tryb koordynacji
		ecp_t.continuous_coordination = false;

		// generacja pierwszego kroku ruchu
		node_counter = 0;

		ecp_t.set_ecp_reply(lib::ECP_ACKNOWLEDGE);
	}

	/**
	 * @brief associated ecp_robot object pointer
	 */
	//boost::shared_ptr<ECP_ROBOT_T> the_robot;
	ECP_ROBOT_T * the_robot;

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	_generator(common::task::_task <ECP_ROBOT_T> & _ecp_task) :
		ecp_mp::generator::generator(*(_ecp_task.sr_ecp_msg)), ecp_t(_ecp_task)//, the_robot(ecp_t.ecp_m_robot)
	{
		if (ecp_t.ecp_m_robot.get()) {
			the_robot = dynamic_cast <ECP_ROBOT_T *> (ecp_t.ecp_m_robot.get());
		}
	}

	/**
	 * @brief Desstructor
	 */
	virtual ~_generator()
	{
	}

	/**
	 * @brief single trajectory node
	 */
	lib::trajectory_description td;
};

typedef _generator <robot::ecp_robot> generator;

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_H */

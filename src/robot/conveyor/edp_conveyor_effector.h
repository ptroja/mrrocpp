// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_conveyor_effector.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Tasmociag
//				- deklaracja klasy edp_conveyor_effector
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------

#ifndef __edp_conveyor_effector_H
#define __edp_conveyor_effector_H

#include "robot/conveyor/sg_conv.h"
#include "base/edp/edp_e_motor_driven.h"
#include "robot/conveyor/const_conveyor.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

/*!
 * @brief Conveyor number of encoder increments per motor revolution
 * @ingroup conveyor
 */
const double INC_PER_REVOLUTION = 4000;

// Klasa reprezentujaca tasmociag.
class effector : public common::motor_driven_effector
{
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	virtual void create_kinematic_models_for_given_robot(void);

public:
	// Konstruktor.
	effector(lib::configurator &_config);

	void set_robot_model(const lib::c_buffer &instruction); // zmiana narzedzia

	void create_threads();

	// Przemieszczenie ramienia.
	void move_arm(const lib::c_buffer &instruction);
	// Odczytanie pozycji ramienia.
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction);
	// Aktualizacja polozenia.


	common::servo_buffer *return_created_servo_buffer();
	void master_order(common::MT_ORDER nm_task, int nm_tryb);
};

} // namespace conveyor
} // namespace edp
} // namespace mrrocpp


#endif

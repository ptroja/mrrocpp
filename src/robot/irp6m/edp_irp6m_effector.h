// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6p_effector.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- deklaracja klasy edp_irp6p_effector
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------


#ifndef __EDP_IRP6_MECHATRONIKA_H
#define __EDP_IRP6_MECHATRONIKA_H

// Klasa edp_irp6s_robot.

#include "base/edp/edp_e_manip.h"
#include "robot/irp6m/const_irp6m.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector : public common::manip_effector
{
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	virtual void create_kinematic_models_for_given_robot(void);

public:
	void set_robot_model(const lib::c_buffer &instruction); // zmiana narzedzia
	void create_threads();

	// Konstruktor.
	effector(lib::configurator &_config);

	void move_arm(const lib::c_buffer &instruction); // przemieszczenie ramienia

	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

	common::servo_buffer *return_created_servo_buffer();
	void master_order(common::MT_ORDER nm_task, int nm_tryb);
};

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif

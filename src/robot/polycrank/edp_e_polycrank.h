// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:		edp_e_polycrank.h
// System:		QNX/MRROC++  v. 6.5
// Opis:		Robot polycrank
//				- deklaracja klasy edp_e_polycrank
//
// Autor:		Mariusz Zbikowski
// Data:		13.01.2011
// ------------------------------------------------------------------------


#ifndef __EDP_E_POLYCRANK_H
#define __EDP_E_POLYCRANK_H

//#include "robot/conveyor/sg_conv.h"
#include "base/edp/edp_e_motor_driven.h"
//#include "robot/conveyor/const_conveyor.h"
#include "base/edp/edp_e_manip.h"
#include "robot/polycrank/sg_polycrank.h"
#include "robot/polycrank/const_polycrank.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

const double INC_PER_REVOLUTION = 4000;

class effector: public common::motor_driven_effector //manip_effector// Klasa reprezentujaca robota IRp-6 na postumencie
{
protected:
	virtual void create_kinematic_models_for_given_robot(void); // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie
public:
	effector(common::shell &_shell); // Konstruktor

	void set_robot_model(const lib::c_buffer &instruction); // Zmiana narzedzia
	//void get_controller_state(lib::c_buffer &instruction);

	void create_threads();
	void move_arm(const lib::c_buffer &instruction); // Przemieszczenie ramienia
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // Odczytanie pozycji ramienia

	//common::servo_buffer *return_created_servo_buffer();
	common::servo_buffer *return_created_servo_buffer();

	void master_order(common::MT_ORDER nm_task, int nm_tryb);
};
} // namespace polycrank
} // namespace edp
} // namespace mrrocpp
/*
namespace mrrocpp {
namespace edp {
namespace conveyor {
const double INC_PER_REVOLUTION = 4000; //Conveyor number of encoder increments per motor revolution

class effector : public common::motor_driven_effector
{
protected:
	virtual void create_kinematic_models_for_given_robot(void); // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie
public:
	effector(common::shell &_shell); // Konstruktor

	void set_robot_model(const lib::c_buffer &instruction); // Zmiana narzedzia

	void create_threads();
	void move_arm(const lib::c_buffer &instruction); // Przemieszczenie ramienia
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // Odczytanie pozycji ramienia.

	// Aktualizacja polozenia.
	common::servo_buffer *return_created_servo_buffer();

	void master_order(common::MT_ORDER nm_task, int nm_tryb);
};

} // namespace conveyor
} // namespace edp
} // namespace mrrocpp
*/

#endif

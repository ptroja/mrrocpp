// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6ot_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robota IRp-6 na torze jezdnym
//				- definicja metod klasy edp_irp6ot_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/edp/reader.h"

// Klasa edp_conveyor_effector.
#include "base/edp/edp_typedefs.h"
#include "base/edp/manip_trans_t.h"
#include "robot/conveyor/edp_conveyor_effector.h"
#include "base/edp/servo_gr.h"
// Model kinematyczny tasmociagu.
#include "robot/conveyor/kinematic_model_conveyor.h"
#include "robot/conveyor/const_conveyor.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace conveyor {

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	motor_driven_effector::hi_create_threads();
}

common::servo_buffer* effector::return_created_servo_buffer()
{
	return new conveyor::servo_buffer(*this);
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::multi_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	motor_driven_effector(_config, lib::conveyor::ROBOT_NAME)
{
	//  Stworzenie listy dostepnych kinematyk.
	number_of_servos = lib::conveyor::NUM_OF_SERVOS;

	create_kinematic_models_for_given_robot();

	reset_variables();
}

/*--------------------------------------------------------------------------*/
void effector::set_robot_model(const lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;
	//printf(" SET ROBOT_MODEL: ");
	switch (instruction.set_robot_model_type)
	{
		case lib::SERVO_ALGORITHM:
			sb->set_robot_model_servo_algorithm(instruction);
			break;

		default: // blad: nie istniejca specyfikacja modelu robota
			// ustawi numer bledu
			motor_driven_effector::set_robot_model(instruction);
	}
}
/*--------------------------------------------------------------------------*/

// Przemieszczenie tasmociagu.
void effector::move_arm(const lib::c_buffer &instruction)
{
	motor_driven_effector::multi_thread_move_arm(instruction);
}

// Odczytanie pozycji tasmociagu.
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{

	//lib::JointArray desired_joints_tmp(lib::MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	if (read_hardware) {
		motor_driven_effector::get_arm_position_read_hardware_sb();
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	common::motor_driven_effector::get_arm_position_get_arm_type_switch(instruction);

	reply.servo_step = step_counter;

}

// Stworzenie modeli kinematyki dla tasmociagu.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::conveyor::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}//: create_kinematic_models_for_given_robot

} // namespace conveyor

namespace common {

// Stworzenie obiektu edp_conveyor_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new conveyor::effector(_config);
}//: return_created_efector

} // namespace common
} // namespace edp
} // namespace mrrocpp

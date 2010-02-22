// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6p_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6p_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6p_tfg/edp_irp6p_tfg_effector.h"
// Kinematyki.

#include "kinematics/irp6p_tfg/kinematic_model_irp6p_tfg.h"

namespace mrrocpp {
namespace edp {
namespace irp6p {



/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	motor_driven_effector::hi_create_threads();
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::multi_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	motor_driven_effector(_config, lib::ROBOT_IRP6P_TFG)
{

	number_of_servos = IRP6P_TFG_NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
}

/*--------------------------------------------------------------------------*/
void effector::set_robot_model(lib::c_buffer &instruction)
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
void effector::move_arm(lib::c_buffer &instruction)
{
	motor_driven_effector::multi_thread_move_arm(instruction);
}

// Odczytanie pozycji tasmociagu.
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{

	//lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	if (read_hardware) {
		motor_driven_effector::get_arm_position_read_hardware_sb();
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	common::motor_driven_effector::get_arm_position_get_arm_type_switch(instruction);

	reply.servo_step = step_counter;

}

common::servo_buffer* effector::return_created_servo_buffer()
{
	return new irp6p::servo_buffer(*this);
}

// Stworzenie modeli kinematyki dla robota IRp-6 na torze.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6p::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);

}

} // namespace irp6ot

namespace common {

// Stworzenie obiektu edp_irp6p_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new irp6p::effector(_config);
}



} // namespace common
} // namespace edp
} // namespace mrrocpp


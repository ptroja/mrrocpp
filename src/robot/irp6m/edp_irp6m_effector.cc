// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6m_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6m_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "robot/irp6m/sg_irp6m.h"
#include "robot/irp6m/edp_irp6m_effector.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/irp6m/kinematic_model_irp6m_with_wrist.h"
#include "robot/irp6m/kinematic_model_irp6m_5dof.h"
#include "base/edp/servo_gr.h"
#include "base/edp/manip_trans_t.h"
#include "base/kinematics/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	motor_driven_effector::hi_create_threads();
}

common::servo_buffer* effector::return_created_servo_buffer()
{
	return new irp6m::servo_buffer(*this);
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::multi_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::irp6m::ROBOT_NAME)
{
	number_of_servos = lib::irp6m::NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
}

/*--------------------------------------------------------------------------*/
void effector::set_robot_model(const lib::c_buffer &instruction)
{
	manip_effector::set_robot_model_with_sb(instruction);
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{ // przemieszczenie ramienia
	// Wypenienie struktury danych transformera na podstawie parametrow polecenia
	// otrzymanego z ECP. Zlecenie transformerowi przeliczenie wspolrzednych

	manip_effector::multi_thread_move_arm(instruction);
}
/*--------------------------------------------------------------------------*/

// sprawdza stan EDP zaraz po jego uruchomieniu


/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia
	//lib::JointArray desired_joints_tmp(lib::MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	//   printf(" GET ARM\n");

	if (read_hardware) {
		motor_driven_effector::get_arm_position_read_hardware_sb();
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	common::manip_effector::get_arm_position_get_arm_type_switch(instruction);

	reply.servo_step = step_counter;
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6m::model_with_wrist());
	add_kinematic_model(new kinematics::irp6m::model_5dof());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace irp6m
namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new irp6m::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


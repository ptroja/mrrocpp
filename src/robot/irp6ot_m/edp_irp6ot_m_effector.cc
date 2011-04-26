// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6ot_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robota IRp-6 na torze jezdnym
//				- definicja metod klasy edp_irp6ot_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "robot/irp6ot_m/edp_irp6ot_m_effector.h"
#include "robot/irp6ot_m/sg_irp6ot_m.h"
// Kinematyki.
#include "robot/irp6ot_m/kinematic_model_irp6ot_with_track.h"
#include "robot/irp6ot_m/kinematic_model_irp6ot_with_wrist.h"
#include "robot/irp6ot_m/kinematic_model_calibrated_irp6ot_with_wrist.h"
#include "base/kinematics/kinematic_model_with_tool.h"
#include "base/edp/edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_m {

common::servo_buffer* effector::return_created_servo_buffer()
{
	return new servo_buffer(*this);
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

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{

	// jesli wlaczono obsluge sily

	vs = (boost::shared_ptr <sensor::force>) sensor::return_created_edp_force_sensor(*this); //!< czujnik wirtualny

	// byY - utworzenie watku pomiarow sily
	new boost::thread(boost::bind(&sensor::force::operator(), vs));

	vs->thread_started.wait();

	motor_driven_effector::hi_create_threads();
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::irp6ot_m::ROBOT_NAME)
{
	number_of_servos = lib::irp6ot_m::NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na torze.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6ot::model_with_wrist(number_of_servos));
	add_kinematic_model(new kinematics::irp6ot::model_with_track(number_of_servos));
	add_kinematic_model(new kinematics::irp6ot::model_calibrated_with_wrist(number_of_servos));
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia
	manip_effector::get_arm_position_with_force_and_sb(read_hardware, instruction);
}
/*--------------------------------------------------------------------------*/

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::multi_thread_master_order(nm_task, nm_tryb);
}

} // namespace irp6ot

namespace common {

// Stworzenie obiektu edp_irp6p_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new irp6ot_m::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


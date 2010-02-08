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

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6_on_track/edp_irp6ot_effector.h"
// Kinematyki.
#include "kinematics/irp6_on_track/kinematic_model_irp6ot_with_track.h"
#include "kinematics/irp6_on_track/kinematic_model_irp6ot_with_wrist.h"
#include "kinematics/irp6_on_track/kinematic_model_calibrated_irp6ot_with_wrist.h"
#include "kinematics/common/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot {

common::servo_buffer* effector::return_created_servo_buffer ()
{
	return new irp6ot::servo_buffer (*this);
}


// Konstruktor.
effector::effector(lib::configurator &_config) :
	irp6s_postument_track_effector(_config, lib::ROBOT_IRP6_ON_TRACK)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	if (is_gripper_active)
		number_of_servos = IRP6_ON_TRACK_NUM_OF_SERVOS;
	else
		number_of_servos = IRP6_ON_TRACK_NUM_OF_SERVOS-1;

	gripper_servo_nr = IRP6OT_GRIPPER_CATCH_AXE;

	reset_variables();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na torze.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6ot::model_with_wrist());
	add_kinematic_model(new kinematics::irp6ot::model_with_track());
	add_kinematic_model(new kinematics::irp6ot::model_calibrated_with_wrist());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);

}


} // namespace irp6ot

namespace common {

// Stworzenie obiektu edp_irp6p_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new irp6ot::effector (_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


/*!
 * @file
 * @brief File contains ecp robot class definition for IRp6 on track manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_m
 */

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
			robot::ecp_robot(lib::irp6ot_m::ROBOT_NAME, lib::irp6ot_m::NUM_OF_SERVOS, lib::irp6ot_m::EDP_SECTION, _config, _sr_ecp),
			kinematics_manager()
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task& _ecp_object) :
	robot::ecp_robot(lib::irp6ot_m::ROBOT_NAME, lib::irp6ot_m::NUM_OF_SERVOS, lib::irp6ot_m::EDP_SECTION, _ecp_object),
			kinematics_manager()
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na torze.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6ot::model_with_wrist(number_of_servos));
	add_kinematic_model(new kinematics::irp6ot::model_with_track(number_of_servos));
	add_kinematic_model(new kinematics::irp6ot::model_calibrated_with_wrist(number_of_servos));
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);

}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp



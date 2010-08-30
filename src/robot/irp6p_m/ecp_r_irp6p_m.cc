/*!
 * @file
 * @brief File contains ecp robot class definition for IRp6 postument manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_m
 */

#include "base/lib/mis_fun.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	robot::ecp_robot(lib::ROBOT_IRP6P_M, IRP6P_M_NUM_OF_SERVOS, EDP_IRP6P_M_SECTION, _config, _sr_ecp), kinematics_manager()
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task& _ecp_object) :
	robot::ecp_robot(lib::ROBOT_IRP6P_M, IRP6P_M_NUM_OF_SERVOS, EDP_IRP6P_M_SECTION, _ecp_object), kinematics_manager()
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6p::model_with_wrist(number_of_servos));
	add_kinematic_model(new kinematics::irp6p::model_5dof(number_of_servos));
	add_kinematic_model(new kinematics::irp6p::model_calibrated_with_wrist(number_of_servos));
	add_kinematic_model(new kinematics::irp6p::model_jacobian_with_wrist(number_of_servos));
	add_kinematic_model(new kinematics::irp6p::model_jacobian_transpose_with_wrist(number_of_servos));
	//add_kinematic_model(new kinematic_model_irp6p_jacobian_with_wrist());

	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

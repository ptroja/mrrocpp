/*!
 * @file
 * @brief File contains ecp robot class definition for IRp6 postument manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_m
 */

#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "robot/irp6p_m/const_irp6p_m.h"

#include "robot/irp6p_m/kinematic_model_irp6p_5dof.h"
#include "robot/irp6p_m/kinematic_model_calibrated_irp6p_with_wrist.h"
#include "robot/irp6p_m/kinematic_model_irp6p_jacobian_with_wrist.h"
#include "robot/irp6p_m/kinematic_model_irp6p_jacobian_transpose_with_wrist.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp::common::robot::ecp_robot(lib::irp6p_m::ROBOT_NAME, lib::irp6p_m::NUM_OF_SERVOS, _config, _sr_ecp)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task_base& _ecp_object) :
	ecp::common::robot::ecp_robot(lib::irp6p_m::ROBOT_NAME, lib::irp6p_m::NUM_OF_SERVOS, _ecp_object)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

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

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

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mathtr.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6_postument/edp_irp6p_effector.h"
// Kinematyki.
#include "kinematics/irp6_postument/kinematic_model_irp6p_with_wrist.h"
#include "kinematics/irp6_postument/kinematic_model_irp6p_5dof.h"
#include "kinematics/irp6_postument/kinematic_model_calibrated_irp6p_with_wrist.h"
#include "kinematics/irp6_postument/kinematic_model_calibrated_correction_matrix_irp6p_with_wrist.h"
#include "kinematics/irp6_postument/kinematic_model_irp6p_jacobian_with_wrist.h"
#include "kinematics/irp6_postument/kinematic_model_irp6p_jacobian_transpose_with_wrist.h"

namespace mrrocpp {
namespace edp {
namespace irp6p {

// Konstruktor.
edp_irp6p_effector::edp_irp6p_effector(configurator &_config) :
	edp_irp6s_postument_track_effector(_config, ROBOT_IRP6_POSTUMENT)
{
}

void edp_irp6p_effector::initialize(void)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	if (is_gripper_active)
		number_of_servos = IRP6_POSTUMENT_NUM_OF_SERVOS;
	else
		number_of_servos = IRP6_POSTUMENT_NUM_OF_SERVOS-1;

	gripper_servo_nr = IRP6P_GRIPPER_CATCH_AXE;

	reset_variables();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void edp_irp6p_effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematic::irp6p::model_with_wrist());
	add_kinematic_model(new kinematic::irp6p::model_5dof());
	add_kinematic_model(new kinematic::irp6p::model_calibrated_with_wrist());
	add_kinematic_model(new kinematic::irp6p::model_calibrated_correction_matrix_with_wrist());
	add_kinematic_model(new kinematic::irp6p::model_jacobian_with_wrist());
	add_kinematic_model(new kinematic::irp6p::model_jacobian_transpose_with_wrist());
	//add_kinematic_model(new kinematic_model_irp6p_jacobian_with_wrist());

	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace irp6p

namespace common {

// Stworzenie obiektu edp_irp6p_effector.
edp_effector* return_created_efector(configurator &_config)
{
	return new irp6p::edp_irp6p_effector (_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


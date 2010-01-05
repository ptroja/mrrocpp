/*!
 * \file kinematic_model.h
 * \brief File containing the declaration of kinematic_model class.
 *
 * \author tkornuta
 * \date Nov 26, 2009
 */

// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6m_with_wrist.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- definicja metod klasy
//				- wykorzystanie nowego stopnia swobody  jako czynnego stopnia swobody
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

#include <math.h>

#include "lib/com_buf.h"
#include "kinematics/spkm/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

kinematic_model::kinematic_model(void)
{
	// Ustawienie etykiety modelu kinematycznego.
	set_kinematic_model_label("Switching to kinematic model with active wrist");

	// Ustawienie parametrow kinematycznych.
	set_kinematic_parameters();
}

void kinematic_model::set_kinematic_parameters(void)
{
}

void kinematic_model::check_motor_position(const double motor_position[])
{
}

void kinematic_model::check_joints(const double q[])
{
}

void kinematic_model::mp2i_transform(const double* local_current_motor_pos, double* local_current_joints)
{
}

void kinematic_model::i2mp_transform(double* local_desired_motor_pos_new, double* local_desired_joints)
{
}

void kinematic_model::direct_kinematics_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame)
{
}

void kinematic_model::inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame)
{
}

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


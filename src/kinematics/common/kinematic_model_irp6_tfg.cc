// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_conveyor.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki tasmociogu
//				- definicja metod klasy
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

#include "lib/com_buf.h"

// Klasa kinematic_model_conveyor.
#include "kinematics/common/kinematic_model_irp6_tfg.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

/* -----------------------------------------------------------------------
 Konstruktor.
 ------------------------------------------------------------------------- */
kinematic_model_irp6_tfg::kinematic_model_irp6_tfg(void)
{
} //: set_kinematic_parameters


/* ------------------------------------------------------------------------
 Sprawdzenie ograniczen na polozenia katowe walow silnikow.
 ------------------------------------------------------------------------ */
void kinematic_model_irp6_tfg::check_motor_position(const lib::MotorArray & motor_position)
{
	if (motor_position[0] < lower_limit_axis) // Kat f8 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_0);
	else if (motor_position[0] > upper_limit_axis) // Kat f8 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_0);
} // end: kinematic_model_conveyor::check_motor_position(const )


/* ------------------------------------------------------------------------
 Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
 ------------------------------------------------------------------------ */
void kinematic_model_irp6_tfg::check_joints(const lib::JointArray & q)
{
	if (isnan(q[0]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA7);
	if (q[0] < lower_limit_joint) // 7 st. swobody
		throw NonFatal_error_2(BEYOND_LOWER_THETA1_LIMIT);

	if (q[0] > upper_limit_joint) // 7 st. swobody
		throw NonFatal_error_2(BEYOND_UPPER_THETA1_LIMIT);

} // end: kinematic_model_conveyor::check_joints(const )


/* ------------------------------------------------------------------------
 Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne
 (mp2i - motor position to internal)
 ------------------------------------------------------------------------ */
void kinematic_model_irp6_tfg::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
	local_current_joints[0] = dir_a_7 * (local_current_motor_pos[0] * local_current_motor_pos[0]) - dir_b_7
			* local_current_motor_pos[0] + dir_c_7;

	// Sprawdzenie obliczonych wartosci wspolrzednych wewnetrznych.
	check_joints(local_current_joints);

}//: mp2i_transform


/* ------------------------------------------------------------------------
 Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
 (i2mp - internal to motor position)
 ------------------------------------------------------------------------ */
void kinematic_model_irp6_tfg::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, lib::JointArray & local_desired_joints)
{

	// Obliczenie kata obrotu walu silnika napedowego chwytaka.
	local_desired_motor_pos_new[0] = inv_a_7 * sqrt(inv_b_7 + inv_c_7 * local_desired_joints[0]) + inv_d_7;

	// Sprawdzenie obliczonych wartosci wspolrzednych wewnetrznych.
	check_joints(local_desired_joints);

	// Sprawdzenie obliczonych wartosci.
	check_motor_position(local_desired_motor_pos_new);

} //: i2mp_transform


} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp


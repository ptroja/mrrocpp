/*!
 * @file
 * @brief File containing the Sarkofag kinematic model class definition.
 *
 * @author tkornuta
 *
 * @ingroup KINEMATICS sarkofag
 */

#include <cmath>

// for MacOS compatibility, where isnan() is implemented as a function in the std:: namespace
// using std::isnan;

#include "base/lib/com_buf.h"

// Klasa kinematic_model_conveyor.
#include "robot/sarkofag/kinematic_model_sarkofag.h"

namespace mrrocpp {
namespace kinematics {
namespace sarkofag {

/* -----------------------------------------------------------------------
 Konstruktor.
 ------------------------------------------------------------------------- */
model::model(void)
{
	// Ustawienie etykiety modelu kinematycznego.
	set_kinematic_model_label("Switching to standard kinematic model");

	// Ustawienie parametrow kinematycznych.
	set_kinematic_parameters();

} //: set_kinematic_parameters

/* -----------------------------------------------------------------------
 Ustawienia wszystkie parametry modelu kinematycznego danego modelu.
 ------------------------------------------------------------------------- */
void model::set_kinematic_parameters(void)
{

	dir_a_7 = -0.00000000283130;
	dir_b_7 = 0.00001451910074;
	dir_c_7 = 0.074;
	inv_a_7 = 0.3531946456e-5;
	inv_b_7 = 0.2622172716e19;
	inv_c_7 = -0.2831300000e20;
	inv_d_7 = -2564.034320;
	gear = 150.0;
	theta = 0.000000e+00;

	synchro_motor_position = 0; // chwytak [-]

	synchro_joint_position = synchro_motor_position - gear * theta;

	/* -----------------------------------------------------------------------
	 Zakresy ruchu walow silnikow w radianach.
	 ------------------------------------------------------------------------- */
	lower_limit_axis = -450.0;
	upper_limit_axis = 450.0;

	/* -----------------------------------------------------------------------
	 Zakresy ruchu poszczegolnych stopni swobody (w radianach lub milimetrach).
	 ------------------------------------------------------------------------- */
	lower_limit_joint = -2.8;
	upper_limit_joint = 2.8;

} // end: set_kinematic_parameters

/* ------------------------------------------------------------------------
 Sprawdzenie ograniczen na polozenia katowe walow silnikow.
 ------------------------------------------------------------------------ */
void model::check_motor_position(const lib::MotorArray & motor_position) const
{
	if (motor_position[0] < lower_limit_axis) // Kat f8 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_0));
	else if (motor_position[0] > upper_limit_axis) // Kat f8 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_0));
} // end: kinematic_model_conveyor::check_motor_position(const )

/* ------------------------------------------------------------------------
 Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
 ------------------------------------------------------------------------ */
void model::check_joints(const lib::JointArray & q) const
{
	if (isnan(q[0]))
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA7));
	if (q[0] < lower_limit_joint) // 7 st. swobody
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA1_LIMIT));

	if (q[0] > upper_limit_joint) // 7 st. swobody
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA1_LIMIT));

} // end: kinematic_model_conveyor::check_joints(const )

/* ------------------------------------------------------------------------
 Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne
 (mp2i - motor position to internal)
 ------------------------------------------------------------------------ */
void model::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{

	local_current_joints[0] = (local_current_motor_pos[0] - synchro_motor_position) / gear + theta;

	// Sprawdzenie obliczonych wartosci.
	check_motor_position(local_current_motor_pos);

	// Sprawdzenie obliczonych wartosci wspolrzednych wewnetrznych.
	check_joints(local_current_joints);

} //: mp2i_transform

/* ------------------------------------------------------------------------
 Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
 (i2mp - internal to motor position)
 ------------------------------------------------------------------------ */
void model::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{

	// Obliczenie kata obrotu walu silnika napedowego chwytaka.
	local_desired_motor_pos_new[0] = gear * local_desired_joints[0] + synchro_joint_position;

	// Sprawdzenie obliczonych wartosci wspolrzednych wewnetrznych.
	check_joints(local_desired_joints);

	// Sprawdzenie obliczonych wartosci.
	check_motor_position(local_desired_motor_pos_new);

} //: i2mp_transform

} // namespace sarkofag
} // namespace kinematic
} // namespace mrrocpp


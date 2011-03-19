/*!
 * @file
 * @brief File containing methods the IRp-6p with wrist (6DOFs) calibrated kinematic model class.
 *
 * @author tkornuta
 * @date 14.02.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS irp6p_m
 */

#include "robot/irp6p_m/kinematic_model_calibrated_irp6p_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6p {

model_calibrated_with_wrist::model_calibrated_with_wrist(int _number_of_servos) :
	model_with_wrist(_number_of_servos)
{
	// Ustawienie etykiety modelu kinematycznego.
	set_kinematic_model_label("Switching to calibrated kinematic model with active wrist");

	// Ustawienie parametrow kinematycznych.
	set_kinematic_parameters();

}

void model_calibrated_with_wrist::set_kinematic_parameters(void)
{
	/* -----------------------------------------------------------------------
	 Numery osi:
	 0 - kolumna obrotowa: os FI
	 1 - ramie dolne:     os TETA
	 2 - ramie gorne:    os ALFA
	 3 - pochylnie kisci: os T
	 4 - obrot kisci:    os V
	 5 - obrot kisci:    os N
	 6 - chwytak
	 ------------------------------------------------------------------------- */

	/* -----------------------------------------------------------------------
	 Dlugosci czlonow robota [m].
	 ------------------------------------------------------------------------- */
	a2 = 0.4596;
	a3 = 0.6729;
	d5 = 0.1903;

	/* -----------------------------------------------------------------------
	 Poprawione wspolczynniki.
	 ------------------------------------------------------------------------- */
	theta[0] = 0.0111;

	sl123 = 78379.4388;
	mi1 = 60059.3311;
	ni1 = -32453.0983;
	theta[1] = 211.8108; // l02

	mi2 = -42863.8177;
	ni2 = -53294.4762;
	theta[2] = 187.6529; // l03

	theta[4] = 0.0903;

	theta[5] = 0.0209;

	/* -----------------------------------------------------------------------
	 Polozenia synchronizacji - odczyty z enkoderow silnikow.
	 ------------------------------------------------------------------------- */
	synchro_motor_position[0] = -7.7597; // kolumna [rad]
	synchro_motor_position[1] = -8.7537; // ramie d. [rad]
	synchro_motor_position[2] = -7.5355; // ramie g. [rad]
	synchro_motor_position[3] = 153.1366; // kisc T [rad]
	synchro_motor_position[4] = 309.5910; // kisc V [rad]
	synchro_motor_position[5] = 796.0265; // kisc N [rad]

	/* -----------------------------------------------------------------------
	 Polozenia synchronizacji we wspolrzednych wewnetrznych - obliczone na podstawie z enkoderow silnikow.
	 ------------------------------------------------------------------------- */
	synchro_joint_position[0] = synchro_motor_position[0] - gear[0] * theta[0];
	synchro_joint_position[1] = synchro_motor_position[1] - gear[1] * theta[1];
	synchro_joint_position[2] = synchro_motor_position[2] - gear[2] * theta[2];
	synchro_joint_position[3] = synchro_motor_position[3] - gear[3] * theta[3];
	synchro_joint_position[4] = synchro_motor_position[4] - gear[4] * theta[4] - synchro_motor_position[3];
	synchro_joint_position[5] = synchro_motor_position[5] - gear[5] * theta[5];
	synchro_joint_position[6] = synchro_motor_position[6] - gear[6] * theta[6];

} //: set_kinematic_parameters

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp

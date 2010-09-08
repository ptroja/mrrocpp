/*!
 * @file
 * @brief File containing the methods of the kinematic_model_calibrated_irp6ot_with_wrist class.
 *
 * @author tkornuta
 * @date 24.02.2007
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS irp6ot_m
 */

#include "robot/irp6ot_m/kinematic_model_calibrated_irp6ot_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot {

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
	 0 - tor jezdny
	 1 - kolumna obrotowa: os FI
	 2 - ramie dolne:     os TETA
	 3 - ramie gorne:    os ALFA
	 4 - pochylnie kisci: os T
	 5 - obrot kisci:    os V
	 7 - obrot kisci:    os N
	 8 - chwytak
	 ------------------------------------------------------------------------- */

	/* -----------------------------------------------------------------------
	 Poprawione dlugosci czlonow robota [m].
	 ------------------------------------------------------------------------- */
	a2 = 0.4647;
	a3 = 0.6748;
	d5 = 0.1967;

	/* -----------------------------------------------------------------------
	 Poprawione wspolczynniki.
	 ------------------------------------------------------------------------- */
	theta[1] = 0.004989;

	sl123 = 79744.63;
	mi2 = 65788.63;
	ni2 = -22524.88;
	theta[2] = 223.9765; // l02

	mi3 = -44612.77;
	ni3 = -52925.22;
	theta[3] = 189.1123; // l03

	theta[5] = 0.000898;
	theta[6] = 0.005605;

	/* -----------------------------------------------------------------------
	 Poprawione polozenia synchronizacji - odczyty z enkoderow silnikow.
	 ------------------------------------------------------------------------- */
	synchro_motor_position[0] = -0.0117; // tor [m]
	synchro_motor_position[1] = -7.185; // kolumna [rad]
	synchro_motor_position[2] = -23.7333; // ramie d. [rad]
	synchro_motor_position[3] = -4.0065; // ramie g. [rad]
	synchro_motor_position[4] = 153.6764; // kisc T [rad]
	synchro_motor_position[5] = 356.0929; // kisc V [rad]
	synchro_motor_position[6] = 791.4409; // kisc N [rad]

	/* -----------------------------------------------------------------------
	 Polozenia synchronizacji we wspolrzednych wewnetrznych - obliczone na podstawie z enkoderow silnikow.
	 ------------------------------------------------------------------------- */
	synchro_joint_position[0] = synchro_motor_position[0] - gear[0] * theta[0];
	synchro_joint_position[1] = synchro_motor_position[1] - gear[1] * theta[1];
	synchro_joint_position[2] = synchro_motor_position[2] - gear[2] * theta[2];
	synchro_joint_position[3] = synchro_motor_position[3] - gear[3] * theta[3];
	synchro_joint_position[4] = synchro_motor_position[4] - gear[4] * theta[4];
	synchro_joint_position[5] = synchro_motor_position[5] - gear[5] * theta[5] - synchro_motor_position[4];
	synchro_joint_position[6] = synchro_motor_position[6] - gear[6] * theta[6];
	synchro_joint_position[7] = synchro_motor_position[7] - gear[7] * theta[7];

}//: set_kinematic_parameters


} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp


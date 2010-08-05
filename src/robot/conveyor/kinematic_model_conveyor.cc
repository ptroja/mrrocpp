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

// Klasa kinematic_model_conveyor.
#include "robot/conveyor/kinematic_model_conveyor.h"

namespace mrrocpp {
namespace kinematics {
namespace conveyor {

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
	// Polozenie synchronizacji.
	synchro_motor_position = 0;
	// Stosunek polozenia walu silnika do polozenia we wsp. wewn (zewn) w metrach.
	motor_to_intext_ratio = 2250;

} // end: set_kinematic_parameters


/* ------------------------------------------------------------------------
 Sprawdzenie ograniczen na polozenia katowe walow silnikow.
 ------------------------------------------------------------------------ */
void model::check_motor_position(const lib::MotorArray & motor_position)
{
	return;
} // end: kinematic_model_conveyor::check_motor_position(const )


/* ------------------------------------------------------------------------
 Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
 ------------------------------------------------------------------------ */
void model::check_joints(const lib::JointArray & q)
{
	return;
} // end: kinematic_model_conveyor::check_joints(const )


/* ------------------------------------------------------------------------
 Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne
 (mp2i - motor position to internal)
 ------------------------------------------------------------------------ */
void model::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
	local_current_joints[0] = local_current_motor_pos[0] / motor_to_intext_ratio;
}//: mp2i_transform


/* ------------------------------------------------------------------------
 Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow
 (i2mp - internal to motor position)
 ------------------------------------------------------------------------ */
void model::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{
	local_desired_motor_pos_new[0] = local_desired_joints[0] * motor_to_intext_ratio;
} //: i2mp_transform


} // namespace conveyor
} // namespace kinematic
} // namespace mrrocpp


// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6ot_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robota IRp-6 na torze jezdnym
//				- definicja metod klasy edp_irp6ot_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6_on_track/edp_irp6ot_effector.h"
// Kinematyki.
#include "edp/irp6_on_track/kinematic_model_irp6ot_with_track.h"
#include "edp/irp6_on_track/kinematic_model_irp6ot_with_wrist.h"
#include "edp/irp6_on_track/kinematic_model_calibrated_irp6ot_with_wrist.h"
#include "edp/irp6_on_track/kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist.h"

extern master_trans_t_buffer mt_tt_obj;

// Konstruktor.
edp_irp6ot_effector::edp_irp6ot_effector () : 
	edp_irp6s_postument_track_effector (ROBOT_IRP6_ON_TRACK)
{
  //  Stworzenie listy dostepnych kinematyk.
  create_kinematic_models_for_given_robot();

	if (is_gripper_active)
		number_of_servos = IRP6_ON_TRACK_NUM_OF_SERVOS;
	else 
		number_of_servos = IRP6_ON_TRACK_NUM_OF_SERVOS-1;

	gripper_servo_nr = IRP6OT_GRIPPER_CATCH_AXE;

	reset_variables();

	
};//: edp_irp6ot_effector


// Stworzenie modeli kinematyki dla robota IRp-6 na torze.
void edp_irp6ot_effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematic_model_irp6ot_with_wrist());
	add_kinematic_model(new kinematic_model_irp6ot_with_track());
	add_kinematic_model(new kinematic_model_calibrated_irp6ot_with_wrist());
	add_kinematic_model(new kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}//: create_kinematic_models_for_given_robot


// Stworzenie obiektu edp_irp6p_effector.
edp_effector* return_created_efector ()
{
	return new edp_irp6ot_effector ();
}//: return_created_efector

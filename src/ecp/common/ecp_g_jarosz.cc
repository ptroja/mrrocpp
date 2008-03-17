// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_postument
//
// Ostatnia modyfikacja: 23.02.2005
// zmiana: funkcja WAIT_FOR_STOP -> chyba dziala.
// autor: tkornuta
// -------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <fstream>
#if defined(__QNXNTO__)
#include <sys/neutrino.h>
#endif /* __QNXNTO__ */

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_jarosz.h"

ecp_delta_generator::ecp_delta_generator(ecp_task& _ecp_task) :
	ecp_generator(_ecp_task)
{
}

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

// ---------------------------------  KONSTRUKTOR  ----------------------------------------------

ecp_linear_generator::ecp_linear_generator(ecp_task& _ecp_task) :
	ecp_delta_generator(_ecp_task)
{
}

ecp_linear_generator::ecp_linear_generator(ecp_task& _ecp_task, trajectory_description tr_des, int mp_communication_mode_arg) :
	ecp_delta_generator(_ecp_task)
{

	mp_communication_mode=mp_communication_mode_arg;

	td = tr_des;

	for (int i = 0; i < MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (int i = 0; i < 3; i++) {
		a_max_zyz[i] = 5.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 5.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 5.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	a_max_zyz[6] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 5.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;
	// 	EDP_command_and_reply_buffer.sr_ecp_msg.message("Skonstruowano obiekt klasy irp6p_irp6p_linear_generator");
}
;

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_linear_generator::first_step()
{

	switch (td.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.get_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.get_arm_type = JOINT;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end : switch (td.arm_type)

	return true;
}
; // end: bool irp6p_irp6p_linear_generator::first_step ( )

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_linear_generator::next_step()
{
	int i; // licznik kolejnych wspolrzednych wektora [0..5]

	// Kontakt z MP
	if (node_counter-1 == td.interpolation_node_no) { // Koniec odcinka

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;

	switch (td.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				the_robot->EDP_data.next_motor_arm_coordinates[i]
						= the_robot->EDP_data.current_motor_arm_coordinates[i] + node_counter*td.coordinate_delta[i]
								/td.interpolation_node_no;
			} // end:for
			break;

		case JOINT:
			for (i=0; i<MAX_SERVOS_NR; i++) {
				the_robot->EDP_data.next_joint_arm_coordinates[i]
						= the_robot->EDP_data.current_joint_arm_coordinates[i] + node_counter*td.coordinate_delta[i]
								/td.interpolation_node_no;
			} // end:for
			break;

		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<6; i++) {
				the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i]
						= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i] + node_counter*td.coordinate_delta[i]
								/td.interpolation_node_no;
			} // end:for
			the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate + node_counter
					*td.coordinate_delta[6]/td.interpolation_node_no;
			break;

		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<6; i++) {
				the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i]
						= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i] + node_counter*td.coordinate_delta[i]
								/td.interpolation_node_no;
			} // end:for
			the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate + node_counter
					*td.coordinate_delta[6]/td.interpolation_node_no;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);

	}// end:switch


	return true;

} // end: bool irp6p_linear_generator::next_step ( )

// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji
// Iinterpolacja funckja liniowa z parabolicznymin odcinkami krzykowliniowymi
// ####################################################################################################

// ---------------------------------  KONSTRUKTOR  ----------------------------------------------

ecp_linear_parabolic_generator::ecp_linear_parabolic_generator(ecp_task& _ecp_task, trajectory_description tr_des, const double *time_a, const double *time_b) :
	ecp_delta_generator(_ecp_task)
{
	td = tr_des;

	for (int i=0; i<MAX_SERVOS_NR; i++) {
		ta[i]=time_a[i];
		if ( (ta[i]=time_a[i]) <=0 || ta[i] > 1)
			throw ECP_error(NON_FATAL_ERROR, INVALID_TIME_SPECIFICATION);
	}

	for (int i=0; i<MAX_SERVOS_NR; i++) {
		tb[i]=time_b[i];
		if ( (tb[i]=time_b[i]) <=0 || tb[i] > 1)
			throw ECP_error(NON_FATAL_ERROR, INVALID_TIME_SPECIFICATION);
	}

	for (int i = 0; i < MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (int i = 0; i < 3; i++) {
		a_max_zyz[i] = 5.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 5.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 5.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}

	a_max_zyz[6] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 5.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;

	// 	EDP_command_and_reply_buffer.sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_linear_parabolic_generator");
}
; // end:konstruktor

// ----------------------------------------------------------------------------------------------
// --------------------------------  funckja liczaca droge  -------------------------------------
// ----------------------------------------------------------------------------------------------

double ecp_linear_parabolic_generator::calculate_s(const double t, const double ta, const double tb)
{
	double s=0;

	if (t <0 || t >1 ){ 
	throw ECP_error(NON_FATAL_ERROR, INVALID_TIME_SPECIFICATION);
}

if (t<ta)
{
	// przyspieszenie=1
	s=0.5*t*t;
}
else if (t<tb)
{
	// przyspieszenie = 1
	s=0.5*ta*ta +ta*(t-ta);
}
else
{ // tb<czas<=1
	// przyspieszenie=-ta/(1-tb) (wynika z rownosci predkosci 1*ta=a*(1-tb) )
	s=0.5*ta*ta + ta*(tb-ta) + ta*(t-tb)-0.5*(ta/(1-tb))*(t-tb)*(t-tb);
}
s=s/ (0.5*ta*ta+ta*(tb-ta)+0.5*(ta*(1-tb)) ); // przyrost/calka z predkosci

return s;

}
; // end : ecp_linear_parabolic_generator::calculate_s

// ----------------------------------------------------------------------------------------------
// --------------------------------- metoda	first_step -----------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_linear_parabolic_generator::first_step()
{

	switch (td.arm_type) {

		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.get_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.get_arm_type = JOINT;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end : switch ( td.arm_type )

	return true;

}
; // end: bool ecp_linear_parabolic_generator::first_step ( )

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_linear_parabolic_generator::next_step()
{
	int i; // licznik kolejnych wspolrzednych wektora [0..5]

	char messg[128]; // komunikat do SR

	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		switch (td.arm_type) {
			case MOTOR:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					prev_s[i]= the_robot->EDP_data.current_motor_arm_coordinates[i];
				} // end:for
				break;

			case JOINT:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					prev_s[i]= the_robot->EDP_data.current_joint_arm_coordinates[i];
				} // end:for
				break;

			case XYZ_EULER_ZYZ:
				for (int i=0; i<6; i++) {
					prev_s[i]= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
				} // end:for
				prev_s[7] = the_robot->EDP_data.current_gripper_coordinate;
				break;
			case XYZ_ANGLE_AXIS:
				for (int i=0; i<6; i++) {
					prev_s[i]= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
				} // end:for
				prev_s[7] = the_robot->EDP_data.current_gripper_coordinate;
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end:switch

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------


	// Kontakt z MP
	if (node_counter-1 == td.interpolation_node_no) { // Koniec odcinka

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;

	double acc[MAX_SERVOS_NR];
	double vel[MAX_SERVOS_NR];
	double vel_avg[MAX_SERVOS_NR];

	switch (td.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				double s = calculate_s((double)node_counter/td.interpolation_node_no, ta[i], tb[i]);
				the_robot->EDP_data.next_motor_arm_coordinates[i]
						= the_robot->EDP_data.current_motor_arm_coordinates[i] + s*td.coordinate_delta[i];
				vel_avg[i] = the_robot->EDP_data.next_motor_arm_coordinates[i] - prev_s[i];
				acc[i] = vel_avg[i] - prev_vel_avg[i];
				vel[i] = acc[i] * node_counter;

				/* if ( fabs(acc[i]*(1/STEP*STEP) ) > a_max_motor[i] )
				 { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
				 sprintf(messg,"Acceleration in axis %d is %f, max. acc = %f",i, fabs(acc[i]), a_max_motor[i]);
				 sr_ecp_msg.message(messg);
				 throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				 } // end : if
				 if ( fabs(vel[i] * (1/STEP) ) > v_max_motor[i] )
				 { // Sprawdzenie przekroczenia dopuszczalnego predkosci
				 sprintf(messg,"Velocity in axis %d is %f, max. vel = %f",i, fabs(vel[i]) , v_max_motor[i]);
				 sr_ecp_msg.message(messg);
				 throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				 } */// end : if

				prev_s[i] = the_robot->EDP_data.next_motor_arm_coordinates[i];
				prev_vel_avg[i] = vel_avg[i];
			} // end:for
			break;

		case JOINT:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				double s = calculate_s((double)node_counter/td.interpolation_node_no, ta[i], tb[i]);
				the_robot->EDP_data.next_joint_arm_coordinates[i]
						= the_robot->EDP_data.current_joint_arm_coordinates[i] + s*td.coordinate_delta[i];
				vel_avg[i] = the_robot->EDP_data.next_joint_arm_coordinates[i] - prev_s[i];
				acc[i] = vel_avg[i] - prev_vel_avg[i];
				vel[i] = acc[i] * node_counter;

				if (fabs(acc[i]*(1/STEP*STEP)) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				if (fabs(vel[i] * (1/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				prev_s[i] = the_robot->EDP_data.next_joint_arm_coordinates[i];
				prev_vel_avg[i] = vel_avg[i];
			} // end:for
			break;

		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<7; i++) {
				double s = calculate_s((double)node_counter/td.interpolation_node_no, ta[i], tb[i]);

				if (i==6) {
					the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate + s
							*td.coordinate_delta[i];
					vel_avg[i] = the_robot->EDP_data.next_gripper_coordinate - prev_s[i];
				} else {
					the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i]
							= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i] + s*td.coordinate_delta[i];
					vel_avg[i] = the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] - prev_s[i];
				}
				acc[i] = vel_avg[i] - prev_vel_avg[i];
				vel[i] = acc[i] * node_counter;

				if (fabs(acc[i]*(1/STEP*STEP)) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				if (fabs(vel[i] * (1/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if

				if (i==6) {
					prev_s[i] = the_robot->EDP_data.next_gripper_coordinate;
				} else {
					prev_s[i] = the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i];
				}

				prev_vel_avg[i] = vel_avg[i];
			} // end:for
			break;

		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<7; i++) {
				double s = calculate_s((double)node_counter/td.interpolation_node_no, ta[i], tb[i]);

				if (i==6) {
					the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate + s
							*td.coordinate_delta[i];
					vel_avg[i] = the_robot->EDP_data.next_gripper_coordinate - prev_s[i];
				} else {
					the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i]
							= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i] + s*td.coordinate_delta[i];
					vel_avg[i] = the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] - prev_s[i];
				}

				acc[i] = vel_avg[i] - prev_vel_avg[i];
				vel[i] = acc[i] * node_counter;

				if (fabs(acc[i]*(1/STEP*STEP)) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				if (fabs(vel[i] * (1/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					prev_s[i] = the_robot->EDP_data.next_gripper_coordinate;
				} else {
					prev_s[i] = the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i];
				}
				prev_vel_avg[i] = vel_avg[i];
			} // end:for
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch


	return true;

}
; // end: bool ecp_linear_parabolic_generator::next_step ( )

// ####################################################################################################
// Klasa bazowa dla generatorow o zadany przyrost polozenia/orientacji
// wykorzystujacych do interpolacji wielomiany
// ####################################################################################################


ecp_polynomial_generator::ecp_polynomial_generator(ecp_task& _ecp_task) :
	ecp_delta_generator(_ecp_task)
{
}
;

// ----------------------------------------------------------------------------------------------
// --------------------------------- metoda	first_step -----------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_polynomial_generator::first_step()
{

	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametr A0 wielomianu, który wymaga znajomoœci pozycji aktualnej ramienia
	first_interval = true;

	switch (td.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.get_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.get_arm_type = JOINT;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV;
			the_robot->EDP_data.set_type = ARM_DV;
			the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end : switch ( td.arm_type )

	return true;
}
; // end: bool ecp_polynomial_generator::first_step ( )

// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji wykorzystuj¹cy do interpolacji wielomian 3 stopnia
// ciaglosc predkosci
// predkosc poczatkowa i koncowa moze byc zadawana
// ####################################################################################################


// ----------------------------------------------------------------------------------------------
// -----------------  konstruktor dla dla zadanych predkosci vp i vk ----------------------------
// ----------------------------------------------------------------------------------------------

ecp_cubic_generator::ecp_cubic_generator(ecp_task& _ecp_task, trajectory_description tr_des, double *vp, double *vk) :
	ecp_polynomial_generator(_ecp_task)
{
	td = tr_des;
	int tf=td.interpolation_node_no;

	for (int i=0; i<MAX_SERVOS_NR; i++) {
		A1[i]=vp[i];
		A2[i]=((3.0*td.coordinate_delta[i])/(tf*tf)) - (2.0*vp[i])/tf - (vk[i]/tf);
		A3[i]=(((-2.0)*td.coordinate_delta[i])/(tf*tf*tf)) + ((vk[i] + vp[i])/(tf*tf));
	} // end:for

	for (int i = 0; i <MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (int i = 0; i < 3; i++) {
		a_max_zyz[i] = 100.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 100.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 10.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 10.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	a_max_zyz[6] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 5.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;
	// 	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_cubic_generator");
}
; // end : konstruktor

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_cubic_generator::next_step()
{
	int i; // licznik kolejnych wspólrzednych wektora [0..MAX_SERVOS_NR]

	char messg[128]; // komunikat do SR


	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {
		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		switch (td.arm_type) {
			case MOTOR:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->EDP_data.current_motor_arm_coordinates[i];
				} // end:for
				break;

			case JOINT:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->EDP_data.current_joint_arm_coordinates[i];
				} // end:for
				break;

			case XYZ_EULER_ZYZ:
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
				} // end:for
				A0[6]= the_robot->EDP_data.current_gripper_coordinate;
				break;
			case XYZ_ANGLE_AXIS:
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
				} // end:for
				A0[6]= the_robot->EDP_data.current_gripper_coordinate;
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------

	// Kontakt z MP
	if (node_counter-1 == td.interpolation_node_no) { // Koniec odcinka
		//     ecp_t.set_ecp_reply (TASK_TERMINATED); // by Y

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;

	double acc[MAX_SERVOS_NR];
	double vel[MAX_SERVOS_NR];

	switch (td.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				acc[i] = ( 2.0*A2[i] + 6.0*A3[i]*(node_counter) ) * ( 1.0
						/ (STEP*the_robot->EDP_data.motion_steps*STEP*the_robot->EDP_data.motion_steps));

				if (fabs(acc[i]) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if

				vel[i] = (A1[i] + 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) ) * ( 1.0
						/ (STEP*the_robot->EDP_data.motion_steps));

				if (fabs(vel[i]) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if

				the_robot->EDP_data.next_motor_arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
						*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
			} // end:for
			break;

		case JOINT:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				acc[i] = ( 2.0*A2[i] + 6.0*A3[i]*(node_counter) ) * ( 1.0 / (STEP*(the_robot->EDP_data.motion_steps)*STEP*(the_robot->EDP_data.motion_steps)));

				if (fabs(acc[i]) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if

				vel[i] =(A1[i] + 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) ) * ( 1.0
						/ (STEP*the_robot->EDP_data.motion_steps));

				if (fabs(vel[i]) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if

				the_robot->EDP_data.next_joint_arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
						*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
			} // end:for
			break;

		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<7; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					the_robot->EDP_data.next_gripper_coordinate = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
							*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
				} else {
					the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]
							*(node_counter*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
				}

			} // end:for
			break;

		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (i=0; i<6; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					the_robot->EDP_data.next_gripper_coordinate = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
							*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
				} else {
					the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]
							*(node_counter*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
				}
			} // end:for
			break;

		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch

	// skopiowac przygotowany rozkaz dla EDP do bufora wysylkowego


	return true;
}
; // end: bool ecp_cubic_generator::next_step ( )

// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji wykorzystujacy do interpolacji wielomian 5 stopnia
// ciaglosc predkosci i przyspieszenia
// predkosc/przyspieszenie poczatkowa/e i koncowa/e moze byc zadawana/e
// ####################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ------------------------------------------------------
// ----------------------------------------------------------------------------------------------

ecp_quintic_generator::ecp_quintic_generator(ecp_task& _ecp_task, trajectory_description tr_des, double *vp, double *vk, double *ap, double *ak) :
	ecp_polynomial_generator(_ecp_task)
{
	td = tr_des;
	int tf=td.interpolation_node_no;

	for (int i=0; i<MAX_SERVOS_NR; i++) {
		A1[i]=vp[i];
		A2[i]=(ap[i]) / 2.0;
		A3[i]=( (20.0*td.coordinate_delta[i]) - ( ( (8.0*vk[i]) + (12.0*vp[i]) )*tf )
				- ( (3.0*ap[i]) - (ak[i])*(tf*tf) ) ) / ( 2.0*(tf*tf*tf) );
		A4[i]=( (-30.0*td.coordinate_delta[i]) + ( ( (14.0*vk[i]) + (16.0*vp[i]) )*tf ) +( (3.0*ap[i]) - (2.0*ak[i])
				*(tf*tf) ) ) / ( 2.0*(tf*tf*tf*tf) );
		A5[i]=( (12.0*td.coordinate_delta[i]) - ( ( (6.0*vk[i]) + (6.0*vp[i]) )*tf ) - (ap[i] - ak[i])*(tf*tf) ) / ( 2.0
				*(tf*tf*tf*tf*tf) );
	} // end:for

	for (int i = 0; i <MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (int i = 0; i < 3; i++) {
		a_max_zyz[i] = 5.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 5.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 5.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	a_max_zyz[6] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 5.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;
	// 	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_quintic_generator");
}
; // end : konstruktor

// ----------------------------------------------------------------------------------------------
// ----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_quintic_generator::next_step()
{

	char messg[128]; // komunikat do SR


	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		switch (td.arm_type) {
			case MOTOR:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->EDP_data.current_motor_arm_coordinates[i];
				} // end:for
				break;

			case JOINT:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->EDP_data.current_joint_arm_coordinates[i];
				} // end:for
				break;

			case XYZ_EULER_ZYZ:
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
				} // end:for
				A0[6]= the_robot->EDP_data.current_gripper_coordinate;
				break;
			case XYZ_ANGLE_AXIS:
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
				} // end:for
				A0[6]= the_robot->EDP_data.current_gripper_coordinate;
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------

	// Kontakt z MP
	if (node_counter-1 == td.interpolation_node_no) { // Koniec odcinka

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;

	double acc[MAX_SERVOS_NR];
	double vel[MAX_SERVOS_NR];

	switch (td.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (int i=0; i<MAX_SERVOS_NR; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
						*(node_counter*node_counter*node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]
						*(node_counter*node_counter*node_counter) + 5.0*A5[i]*(node_counter*node_counter*node_counter
						*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				the_robot->EDP_data.next_motor_arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
						*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]*(node_counter
						*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter*node_counter
						*node_counter*node_counter);
			} // end:for
			break;

		case JOINT:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (int i=0; i<MAX_SERVOS_NR; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
						*(node_counter*node_counter*node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]
						*(node_counter*node_counter*node_counter) + 5.0*A5[i]*(node_counter*node_counter*node_counter
						*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				the_robot->EDP_data.next_joint_arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
						*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]*(node_counter
						*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter*node_counter
						*node_counter*node_counter);
			} // end:for
			break;

		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (int i=0; i<6; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
						*(node_counter*node_counter*node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]
						*(node_counter*node_counter*node_counter) + 5.0*A5[i]*(node_counter*node_counter*node_counter
						*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					the_robot->EDP_data.next_gripper_coordinate = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
							*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]*(node_counter
							*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter*node_counter
							*node_counter*node_counter);
				} else {
					the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]
							*(node_counter*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]
							*(node_counter*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter
							*node_counter*node_counter*node_counter);
				}

			} // end:for
			break;

		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = SET;
			the_robot->EDP_data.set_type = ARM_DV; // ARM
			the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			for (int i=0; i<6; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
						*(node_counter*node_counter*node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]
						*(node_counter*node_counter*node_counter) + 5.0*A5[i]*(node_counter*node_counter*node_counter
						*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					the_robot->EDP_data.next_gripper_coordinate = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
							*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]*(node_counter
							*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter*node_counter
							*node_counter*node_counter);
				} else {
					the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]
							*(node_counter*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]
							*(node_counter*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter
							*node_counter*node_counter*node_counter);
				}

			} // end:for
			break;

		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch

	// skopiowaæ przygotowany rozkaz dla EDP do bufora wysylkowego


	return true;

}
; // end: bool ecp_quintic_generator::next_step ( )

// ####################################################################################################
// ############################     Odtwarzanie listy pozycji    ######################################
// ####################################################################################################


ecp_spline_generator::ecp_spline_generator(ecp_task& _ecp_task) :
	ecp_teach_in_generator(_ecp_task)
{
}
;

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, z rozpedzaniem i hamowaniem miedzy pozycjami,
// z dokladna zadana pozycja koncowa
// ####################################################################################################


// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ------------------------------------------------------
// ----------------------------------------------------------------------------------------------

ecp_parabolic_teach_in_generator::ecp_parabolic_teach_in_generator(ecp_task& _ecp_task, double interval = 0.02) :
	ecp_spline_generator(_ecp_task)
{
	INTERVAL = interval; // Dlugosc okresu interpolacji w [sek]
	int i;

	for (i = 0; i <MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (i = 0; i < 3; i++) {
		a_max_zyz[i] = 5.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 5.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 5.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	// 	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_parabolic_teach_in_generator");
	a_max_zyz[6] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 5.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;
}
; // end: ecp_parabolic_teach_in_generator::ecp_parabolic_teach_in_generator()

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_parabolic_teach_in_generator::first_step()
{

	// Poniewaz ten generator wykonuje ruch tylko do kolejnej pozycji na liscie,
	// nie informuje MP o skonczeniu sie listy.

	// Zlecenie odczytu aktualnego polozenia ramienia

	if (!is_pose_list_element())
		return false;
	// Pobranie kolejnej pozycji z listy i wstawienie danych do generatora
	// Na podstawie odczytanej aktualnej pozycji oraz kolejnej pozycji na liscie
	// wyznaczane beda przedzialy interpolacji, czyli makrokroki do realizacji przez EDP.
	get_pose(tip);
	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametry ruchu
	first_interval = true;
	switch (tip.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = MOTOR;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = JOINT;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;
}
; // end: bool ecp_parabolic_teach_in_generator::first_step ( )

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_parabolic_teach_in_generator::next_step()
{
	double Delta; // roznica polozen aktulanego  i zadanego

	char messg[128]; // komunikat do SR
	int i; // Licznik

	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		// Wyznaczenie liczby przedzialow interpolacji oraz pozostalych parametrow ruchu
		number_of_intervals = (int) ceil(tip.motion_time/INTERVAL);
		if ( (number_of_intervals % 2) == 1)
			number_of_intervals++;

		half_number_of_intervals = number_of_intervals / 2;
		// Wyznaczenie przyspieszen oraz sprawdzenie,
		// czy nie zostaly przekroczone wartosci maksymalne przyspieszen i predkosci
		switch (tip.arm_type) {
			case MOTOR:
				the_robot->EDP_data.set_arm_type = MOTOR;
				for (i=0; i <MAX_SERVOS_NR; i++) {
					a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i])
							/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if ( 0.5*fabs(a[i] * tip.motion_time) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i+1, 0.5 * a[i] * tip.motion_time, v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case JOINT:
				the_robot->EDP_data.set_arm_type = JOINT;
				for (i=0; i <MAX_SERVOS_NR; i++) {
					a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_joint_arm_coordinates[i])
							/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if ( 0.5*fabs(a[i] * tip.motion_time) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i+1, 0.5*a[i] * tip.motion_time, v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case XYZ_EULER_ZYZ:
				the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
				for (i=0; i < 7; i++) {
					if (i==6) {
						Delta = tip.coordinates[i] - the_robot->EDP_data.current_gripper_coordinate;
					} else {
						Delta = tip.coordinates[i] - the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
					}

					if (i > 2 && i < 6) // Wymuszenie ruchu po krotszym luku okregu
						if (Delta > M_PI) // przy zamianie kata orientacji
							Delta -= 2.0*M_PI;
					if (Delta < -M_PI)
						Delta += 2.0*M_PI;
					a[i] = 4.0*(Delta)/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in coordinate %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if ( 0.5*fabs(a[i] * tip.motion_time) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5*a[i]
								* tip.motion_time, v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case XYZ_ANGLE_AXIS:
				the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
				for (i=0; i < 7; i++) {
					if (i==6) {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_gripper_coordinate)
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					} else {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i])
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					}

					if (fabs(a[i]) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5* a[i]
								* tip.motion_time, v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if ( 0.5*fabs(a[i] * tip.motion_time) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5*a[i]
								* tip.motion_time, v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
		the_robot->EDP_data.instruction_type = SET;
		the_robot->EDP_data.set_type = ARM_DV; // ARM
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.next_interpolation_type = MIM;
		the_robot->EDP_data.motion_steps = (WORD) (INTERVAL/STEP);
		the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-2;
		first_interval = false;

	}

	// (ten, do ktorego zmierza ramie)

	if (node_counter <= number_of_intervals) {
		// Obliczenie polozenia na podstawie danych interpolacyjnych (zgodnie z uplywem czasu)
		switch (tip.arm_type) {
			case MOTOR:

				for (i = 0; i <MAX_SERVOS_NR; i++) {
					if (node_counter < half_number_of_intervals)
						the_robot->EDP_data.next_motor_arm_coordinates[i]
								= the_robot->EDP_data.current_motor_arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->EDP_data.next_motor_arm_coordinates[i] = tip.coordinates[i] - 0.5 * a[i]
								* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
								- node_counter*INTERVAL);
				}
				break;
			case JOINT:
				for (i = 0; i <MAX_SERVOS_NR; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->EDP_data.next_joint_arm_coordinates[i]
								= the_robot->EDP_data.current_joint_arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->EDP_data.next_joint_arm_coordinates[i] = tip.coordinates[i] - 0.5 * a[i]
								* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
								- node_counter*INTERVAL);
				break;
			case XYZ_EULER_ZYZ:
				for (i = 0; i < 6; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i]
								= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = tip.coordinates[i] - 0.5 * a[i]
								* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
								- node_counter*INTERVAL);

				if (node_counter < half_number_of_intervals)
					the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate + 0.5
							* a[6]* node_counter*INTERVAL*node_counter*INTERVAL;
				else
					the_robot->EDP_data.next_gripper_coordinate = tip.coordinates[6] - 0.5 * a[6]
							* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
							- node_counter*INTERVAL);

				break;
			case XYZ_ANGLE_AXIS:
				for (i = 0; i < 6; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i]
								= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = tip.coordinates[i] - 0.5 * a[i]
								* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
								- node_counter*INTERVAL);

				if (node_counter < half_number_of_intervals)
					the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate + 0.5
							* a[6]* node_counter*INTERVAL*node_counter*INTERVAL;
				else
					the_robot->EDP_data.next_gripper_coordinate = tip.coordinates[6] - 0.5 * a[6]
							* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
							- node_counter*INTERVAL);

				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
	}

	// Czy skonczono interpolacje oraz zniwelowano blad interpolacji?
	if (node_counter <= number_of_intervals) {
		// Ruch do kolejnej pozycji na liscie pozycji nauczonych nie zostal
		// zakonczony - informacja dla Move()
		//    the_robot->create_command (); // Przygotowanie rozkazu dla EDP
		// Rozkaz zostanie wykonany przez Move i odpowiednie dane
		// zostana zaktualizowane w robocie
		return true;
	} else {
		// Po przejsciu wszystkich wezlow interpolacji nalezy przestawic wskaznik listy pozycji
		// nauczonych na nastepna pozycje
		next_pose_list_ptr();
		// Ruch do kolejnej pozycji na liscie zakonczono - informacja dla Move()
		return false;
	}
}
; // end: bool ecp_parabolic_teach_in_generator::next_step ( )

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, wykorzystywany do kalibracji
// ####################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ------------------------------------------------------
// ----------------------------------------------------------------------------------------------

ecp_calibration_generator::ecp_calibration_generator(ecp_task& _ecp_task, double interval) :
	ecp_spline_generator(_ecp_task)
{
	INTERVAL = interval; // Dlugosc okresu interpolacji w [sek]
	int i; // Licznik

	for (i = 0; i <MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 200.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (i = 0; i < 3; i++) {
		a_max_zyz[i] = 1000.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 1.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 1000.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 1.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	//  	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_calibration_generator");
	a_max_zyz[6] = 1000.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 1000.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;
}
; // end: ecp_calibration_generator::ecp_calibration_generator()

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_calibration_generator::first_step()
{

	// Poniewaz ten generator wykonuje ruch tylko do kolejnej pozycji na liscie,
	// nie informuje MP o skonczeniu sie listy. Po skonczeniu listy potrzebny jest
	// jeszcze kontakt z UI, aby zapytac, czy zapamietac dane kalibracyjne. Dopiero
	// wtedy mozna poinformowac MP o skonczeniu sie listy.

	// Zlecenie odczytu aktualnego polozenia ramienia

	if (!is_pose_list_element())
		return false;
	// Pobranie kolejnej pozycji z listy i wstawienie danych do generatora
	// Na podstawie odczytanej aktualnej pozycji oraz kolejnej pozycji na liscie
	// wyznaczane beda przedzialy interpolacji, czyli makrokroki do realizacji przez EDP.
	get_pose(tip);
	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametry ruchu
	first_interval = true;
	switch (tip.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = MOTOR;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = JOINT;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;

}
; // end: bool ecp_calibration_generator::first_step ( )


// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_calibration_generator::next_step()
{

	char messg[128]; // komunikat do SR
	int i; // licznik

	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		// Wyznaczenie liczby przedzialow interpolacji oraz pozostalych parametrow ruchu
		number_of_intervals = (int) ceil(tip.motion_time/INTERVAL);
		if ( (number_of_intervals % 2) == 1)
			number_of_intervals++;

		half_number_of_intervals = number_of_intervals / 2;
		// Wyznaczenie przyspieszen oraz sprawdzenie,
		// czy nie zostaly przekroczone wartosci maksymalne przyspieszen i predkosci
		switch (tip.arm_type) {
			case MOTOR:
				the_robot->EDP_data.set_arm_type = MOTOR;
				for (i=0; i <MAX_SERVOS_NR; i++) {
					a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i])
							/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if (fabs(0.5* a[i] * tip.motion_time) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i+1, 0.5* a[i] * tip.motion_time, v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case JOINT:
				the_robot->EDP_data.set_arm_type = JOINT;
				for (i=0; i <MAX_SERVOS_NR; i++) {
					a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_joint_arm_coordinates[i])
							/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if (fabs(0.5* a[i] * tip.motion_time) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i+1, 0.5* a[i] * tip.motion_time, v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case XYZ_EULER_ZYZ:
				the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
				for (i=0; i < 7; i++) {
					if (i==6) {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_gripper_coordinate)
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					} else {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i])
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					}

					if (fabs(a[i]) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in coordinate %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if (fabs(0.5* a[i] * tip.motion_time) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5* a[i]
								* tip.motion_time, v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case XYZ_ANGLE_AXIS:
				the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
				for (i=0; i < 7; i++) {
					if (i==6) {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_gripper_coordinate)
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					} else {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i])
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					}
					if (fabs(a[i]) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5* a[i]
								* tip.motion_time, v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if (fabs(0.5* a[i] * tip.motion_time) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5* a[i]
								* tip.motion_time, v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
		the_robot->EDP_data.instruction_type = SET;
		the_robot->EDP_data.set_type = ARM_DV; // ARM
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.next_interpolation_type = MIM;
		the_robot->EDP_data.motion_steps = (WORD) (INTERVAL/STEP);
		the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-2;
		first_interval = false;

	}

	// (ten, do ktorego zmierza ramie)

	if (node_counter <= number_of_intervals)
		// Obliczenie polozenia na podstawie danych interpolacyjnych (zgodnie z uplywem czasu)
		switch (tip.arm_type) {
			case MOTOR:
				for (i = 0; i <MAX_SERVOS_NR; i++) {
					if (node_counter < half_number_of_intervals)
						the_robot->EDP_data.next_motor_arm_coordinates[i]
								= the_robot->EDP_data.current_motor_arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->EDP_data.next_motor_arm_coordinates[i]
								= the_robot->EDP_data.current_motor_arm_coordinates[i] + a[i] * (node_counter*INTERVAL
										*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
										-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
				}
				break;
			case JOINT:
				for (i = 0; i <MAX_SERVOS_NR; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->EDP_data.next_joint_arm_coordinates[i]
								= the_robot->EDP_data.current_joint_arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->EDP_data.next_joint_arm_coordinates[i]
								= the_robot->EDP_data.current_joint_arm_coordinates[i] + a[i] * (node_counter*INTERVAL
										*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
										-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
				break;
			case XYZ_EULER_ZYZ:
				for (i = 0; i < 6; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i]
								= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i]
								= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i] + a[i] * (node_counter
										*INTERVAL*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter
										*INTERVAL -0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);

				if (node_counter < half_number_of_intervals)
					the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[6]
							= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[6] + 0.5* a[6]* node_counter*INTERVAL
									*node_counter*INTERVAL;
				else
					the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[6]
							= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[6] + a[6] * (node_counter*INTERVAL
									*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
									-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);

				break;
			case XYZ_ANGLE_AXIS:
				for (i = 0; i < 6; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i]
								= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i]
								= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i] + a[i] * (node_counter*INTERVAL
										*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
										-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);

				if (node_counter < half_number_of_intervals)
					the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[6]
							= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[6] + 0.5* a[6]* node_counter*INTERVAL
									*node_counter*INTERVAL;
				else
					the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[6]
							= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[6] + a[6] * (node_counter*INTERVAL
									*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
									-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);

				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
	else { // Zniwelowanie ewentualnego bledu miedzy efektem interpolacji a faktyczna pozycja nauczona - celem ruchu
		the_robot->EDP_data.motion_steps = 25;
		the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps;
		switch (tip.arm_type) {
			case MOTOR:
				memcpy(the_robot->EDP_data.next_motor_arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case JOINT:
				memcpy(the_robot->EDP_data.next_joint_arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case XYZ_EULER_ZYZ:
				memcpy(the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case XYZ_ANGLE_AXIS:
				memcpy(the_robot->EDP_data.next_XYZ_AA_arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
	}

	// Czy skonczono interpolacje oraz zniwelowano blad interpolacji?
	if (node_counter <= number_of_intervals+1) {
		// Ruch do kolejnej pozycji na liscie pozycji nauczonych nie zostal
		// zakonczony - informacja dla Move()
		//   the_robot->create_command (); // Przygotowanie rozkazu dla EDP
		// Rozkaz zostanie wykonany przez Move i odpowiednie dane
		// zostana zaktualizowane w robocie
		return true;
	} else {
		// Po przejsciu wszystkich wezlow interpolacji nalezy przestawic wskaznik listy pozycji
		// nauczonych na nastepna pozycje
		next_pose_list_ptr();
		// Ruch do kolejnej pozycji na liscie zakonczono - informacja dla Move()
		return false;
	}
}
; // end: bool ecp_calibration_generator::next_step ( )

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 3 stopnia,
// z rozpedzaniem i hamowaniem miedzy pozycjami
// ####################################################################################################


ecp_cubic_spline_generator::ecp_cubic_spline_generator(ecp_task& _ecp_task, double interval = 0.02) :
	ecp_spline_generator(_ecp_task)
{
	INTERVAL = interval; // Dlugosc okresu interpolacji w [sek]
	int i;

	for (i = 0; i <MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (i = 0; i < 3; i++) {
		a_max_zyz[i] = 5.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 5.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 5.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}

	a_max_zyz[6] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 5.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;
	// 	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_cubic_spline_generator");

}
; // end : ecp_cubic_spline_generator::ecp_cubic_spline_generator

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_cubic_spline_generator::first_step()
{

	if (is_pose_list_element()) {
	} else {

		return false;
	}

	// Pobranie kolejnej pozycji z listy i wstawienie danych do generatora
	// Na podstawie odczytanej aktualnej pozycji oraz kolejnej pozycji na liscie
	// wyznaczane beda przedzialy interpolacji, czyli makrokroki do realizacji przez EDP.
	get_pose(tip);
	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametry ruchu
	first_interval = true;
	switch (tip.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = MOTOR;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = JOINT;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;

}
; // end : ecp_cubic_spline_generator::first_step()

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_cubic_spline_generator::next_step()
{

	char messg[128]; // komunikat do SR
	int i; // Licznik

	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		// Wyznaczenie liczby przedzialow interpolacji oraz pozostalych parametrow ruchu
		number_of_intervals = (int) ceil(tip.motion_time/INTERVAL);

		switch (tip.arm_type) {
			case MOTOR:
				the_robot->EDP_data.set_arm_type = MOTOR;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->EDP_data.current_motor_arm_coordinates[i];
					A2[i]=(3.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals);
					A3[i]=(-2.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
				} // end:for
				break;

			case JOINT:
				the_robot->EDP_data.set_arm_type = JOINT;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->EDP_data.current_joint_arm_coordinates[i];
					A2[i]=(3.0*(tip.coordinates[i] - the_robot->EDP_data.current_joint_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals);
					A3[i]=(-2.0*(tip.coordinates[i] - the_robot->EDP_data.current_joint_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
				} // end:for
				break;

			case XYZ_EULER_ZYZ:
				the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
					A2[i]=(3.0*(tip.coordinates[i] - the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals);
					A3[i]=(-2.0*(tip.coordinates[i] - the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
				} // end:for

				A0[6]= the_robot->EDP_data.current_gripper_coordinate;
				A2[6]=(3.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals);
				A3[6]=(-2.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals);
				break;
			case XYZ_ANGLE_AXIS:
				the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
					A2[i]=(3.0*(tip.coordinates[i] - the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals);
					A3[i]=(-2.0*(tip.coordinates[i] - the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
				} // end:for
				A0[6]= the_robot->EDP_data.current_gripper_coordinate;
				A2[6]=(3.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals);
				A3[6]=(-2.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals);
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		the_robot->EDP_data.instruction_type = SET;
		the_robot->EDP_data.set_type = ARM_DV; // ARM
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.next_interpolation_type = MIM;
		the_robot->EDP_data.motion_steps = (WORD) (INTERVAL/STEP);
		the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-2;

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------


	// (ten, do ktorego zmierza ramie)

	// ---------------------------------  czy obliczono polozenie dla wszystkich wezlow?    ---------------------------------------

	if (node_counter <= number_of_intervals) {
		// Tablice do przechowywania punktowych wartosci przyspieszen i predkosci
		double acc[MAX_SERVOS_NR];
		double vel[MAX_SERVOS_NR];

		// Obliczenie polozenia na podstawie danych interpolacyjnych (zgodnie z uplywem czasu)
		switch (tip.arm_type) {
			case MOTOR:
				for (i = 0; i <MAX_SERVOS_NR; i++) {
					acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					the_robot->EDP_data.next_motor_arm_coordinates[i] = A0[i] + A2[i]*(node_counter*node_counter)
							+ A3[i]*(node_counter*node_counter*node_counter);
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case JOINT:
				for (i=0; i <MAX_SERVOS_NR; i++) {
					acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					the_robot->EDP_data.next_joint_arm_coordinates[i] = A0[i] + A2[i]*(node_counter*node_counter)
							+ A3[i]*(node_counter*node_counter*node_counter);
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case XYZ_EULER_ZYZ:
				for (i = 0; i < 7; i++) {
					acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->EDP_data.next_gripper_coordinate = A0[i] + A2[i]*(node_counter*node_counter) + A3[i]
								*(node_counter*node_counter*node_counter);
					} else {
						the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = A0[i] + A2[i]*(node_counter*node_counter)
								+ A3[i]*(node_counter*node_counter*node_counter);
					}
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case XYZ_ANGLE_AXIS:
				for (i = 0; i < 7; i++) {
					acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->EDP_data.next_gripper_coordinate = A0[i] + A2[i]*(node_counter*node_counter) + A3[i]
								*(node_counter*node_counter*node_counter);
					} else {
						the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = A0[i] + A2[i]*(node_counter*node_counter)
								+ A3[i]*(node_counter*node_counter*node_counter);
					}
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch

	} // end : if (node_counter <= number_of_intervals)

	// -----------------------------------------------------------------------------------------------------------------------------------------


	// Czy skonczono interpolacje oraz zniwelowano blad interpolacji?
	if (node_counter <= number_of_intervals) {
		// Ruch do kolejnej pozycji na liscie pozycji nauczonych nie zostal
		// zakonczony - informacja dla Move()
		//    the_robot->create_command (); // Przygotowanie rozkazu dla EDP
		//    	sr_ecp_msg.message("koncowka next_step - if ( node_counter <= number_of_intervals 2)");
		// Rozkaz zostanie wykonany przez Move i odpowiednie dane
		// zostana zaktualizowane w robocie
		return true;
	}

	else {
		// Po przejsciu wszystkich wezlow interpolacji nalezy przestawic wskaznik listy pozycji
		// nauczonych na nastepna pozycje
		next_pose_list_ptr();

		// Ruch do kolejnej pozycji na liscie zakonczono - informacja dla Move()
		return false;
	}
}
; // end: bool ecp_cubic_spline_generator::next_step ( )

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 3 stopnia
// ####################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ----------------------------------------------------
// ----------------------------------------------------------------------------------------------

ecp_smooth_cubic_spline_generator::ecp_smooth_cubic_spline_generator(ecp_task& _ecp_task, double *vp, double *vk, double interval = 0.02) :
	ecp_spline_generator(_ecp_task)
{
	INTERVAL = interval; // Dlugosc okresu interpolacji w [sek]
	int i;
	build_coeff = true;

	// Tworzymy macierze:
	y(pose_list_length() + 1, 6);
	t(pose_list_length() + 1, 6);
	a(pose_list_length() + 1, 6);

	for (i = 0; i <MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (i = 0; i < 3; i++) {
		a_max_zyz[i] = 5.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 5.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 5.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	// 	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_smooth_cubic_spline_generator");
	a_max_zyz[6] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 5.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;
}
; // end : ecp_smooth_cubic_spline_generator::ecp_smooth_cubic_spline_generator

// ----------------------------------------------------------------------------------------------
// ---------------------------metoda BuildCoeff -------------------------------------------
// ----------------------------------------------------------------------------------------------

void ecp_smooth_cubic_spline_generator::Build_Coeff(double *tt, double *yy, int nn, double vvp, double vvk, double *aa)
{
	sr_ecp_msg.message("Metoda Build_Coeff wyznacza przyspieszenia w punktach wezlowych");
	int i, k;
	double p, qn, sig, un;

	double *uu = new double[nn-2];

	aa[0]= -0.5;
	uu[0]=( 3.0 /(tt[1]-tt[0]) ) * ( (yy[1]-yy[0]) / (tt[1]-tt[0])-vvp);

	for (i=1; i<=nn-2; i++) { // petla algorytmu trojdiagonalnego

		sig=(tt[i]-tt[i-1])/(tt[i+1]-tt[i-1]);
		p=sig*aa[i-1]+2.0;
		aa[i]=(sig-1.0)/p;
		uu[i]=(yy[i+1]-yy[i])/(tt[i+1]-tt[i]) - (yy[i]-yy[i-1])/(tt[i]-tt[i-1]);
		uu[i]=(6.0*uu[i]/(tt[i+1]-tt[i-1])-sig*uu[i-1])/p;
	}

	qn=0.5;
	un=( 3.0 /(tt[nn-1]-tt[nn-2]) )*(vvk-(yy[nn-1]-yy[nn-2]) / (tt[nn-1]-tt[nn-2]));
	aa[nn-1]=(un-qn*uu[nn-2])/(qn*aa[nn-2]+1.0);

	for (k=nn-2; k>=0; k--) // petla wstecznego zastepowania algorytmu trojdiagonalnego
	{
		aa[k]=aa[k]*aa[k+1]+uu[k];
	}

	delete [] uu;

}
; // end :	 ecp_smooth_cubic_spline_generator::Build_Coeff

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_smooth_cubic_spline_generator::first_step()
{

	if (is_pose_list_element()) {
	} else {

		return false;
	}

	// Zlecenie odczytu aktualnego polozenia ramienia

	// Pobranie kolejnej pozycji z listy i wstawienie danych do generatora
	// Na podstawie odczytanej aktualnej pozycji oraz kolejnej pozycji na liscie
	// wyznaczane beda przedzialy interpolacji, czyli makrokroki do realizacji przez EDP.
	get_pose(tip);
	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametry ruchu
	first_interval = true;
	switch (tip.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = MOTOR;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = JOINT;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch


}
; // end :	 ecp_smooth_cubic_spline_generator::first_step

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_smooth_cubic_spline_generator::next_step()
{

	char messg[128]; // komunikat do SR
	int i; // Licznik

	// ---------------------------------   BUILD COEFF    ---------------------------------------
	if (build_coeff) {

		j=0;

		switch (tip.arm_type) {
			case MOTOR:
				the_robot->EDP_data.set_arm_type = MOTOR;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					y(0, i) = the_robot->EDP_data.current_motor_arm_coordinates[i];
					t(0, i) = (double) 0;
				} // end:for
				break;

			case JOINT:
				the_robot->EDP_data.set_arm_type = JOINT;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					y(0, i) = the_robot->EDP_data.current_joint_arm_coordinates[i];
					t(0, i) = (double) 0;
				} // end:for
				break;

			case XYZ_EULER_ZYZ:
				the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
				for (int i=0; i<7; i++) {
					if (i==6) {
						y(0, i) = the_robot->EDP_data.current_gripper_coordinate;
					} else {
						y(0, i) = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
					}
					t(0, i) = (double) 0;
				} // end:for
				break;

			case XYZ_ANGLE_AXIS:
				the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
				for (int i=0; i<7; i++) {
					if (i==6) {
						y(0, i) = the_robot->EDP_data.current_gripper_coordinate;
					} else {
						y(0, i) = the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
					}
					t(0, i) = (double) 0;
					t(0, i) = (double) 0;
				} // end:for
				break;

			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		// Nastepne wartosci przemieszczen wraz z czasami

		for (j=1; j<=pose_list_length(); j++) {

			switch (tip.arm_type) {
				case MOTOR:
					the_robot->EDP_data.set_arm_type = MOTOR;
					for (int i=0; i<MAX_SERVOS_NR; i++) {

						y(j, i) = tip.coordinates[i];
						t(j, i) = (double) ( t(j-1 ,i) + (tip.motion_time/INTERVAL) );
					} // end:for
					break;

				case JOINT:
					the_robot->EDP_data.set_arm_type = JOINT;
					for (int i=0; i<MAX_SERVOS_NR; i++) {
						y(j, i) = tip.coordinates[i];
						t(j, i) = (double) ( t(j-1 ,i) + (tip.motion_time/INTERVAL) );
					} // end:for
					break;

				case XYZ_EULER_ZYZ:
					the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
					for (int i=0; i<7; i++) {
						y(j, i) = tip.coordinates[i];
						t(j, i) = (double) ( t(j-1 ,i) + (tip.motion_time/INTERVAL) );
					} // end:for
					break;

				case XYZ_ANGLE_AXIS:
					the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
					for (int i=0; i<7; i++) {
						y(j, i) = tip.coordinates[i];
						t(j, i) = (double) ( t(j-1 ,i) + (tip.motion_time/INTERVAL) );
					} // end:for
					break;

				default:
					throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}// end:switch

			next_pose_list_ptr(); // do nastepnej pozycji na liscie
			if (is_pose_list_element() ) {
				get_pose(tip);
			} // end : if

		}// end : for (j=1; j<=pose_list_length(); j++)

		// // // // // // // // // // // // // // // // // // // // // // // // // // // // /
		// ALGORYTM OBLICZANIA WSPOLCZYNNIKOW WIELOMIANU
		// // // // // // // // // // // // // // // // // // // // // // // // // // // // /

		for (int i=0; i<MAX_SERVOS_NR; i++) { // DLA KAZDEJ WSPOLRZEDNEJ BUDUJEMY WSPOLCZYNNIKI METODA BUILD_COEFF

			double *t_temp = new double[pose_list_length() + 1];
			double *y_temp = new double[pose_list_length() + 1];
			double *a_temp = new double[pose_list_length() + 1];

			for (int z=0; z<=pose_list_length(); z++) {
				t_temp[z] = t(z, i);
				y_temp[z] = y(z, i);
			}// end:for

			Build_Coeff(t_temp, y_temp, pose_list_length()+1, vp[i], vk[i], a_temp);

			for (int z=0; z<=pose_list_length(); z++) {
				a(z, i)= a_temp[z];
			}// end:for

			delete [] t_temp;
			delete [] y_temp;
			delete [] a_temp;

		} // end:for DLA KAZDEJ WSPOLRZEDNEJ

		j=0;

		build_coeff = false;
		initiate_pose_list(); // powrot do pierwszej pozycji na liscie
		if (is_pose_list_element() ) {
			get_pose(tip);
		} // end : if

	} // end:if BUILD COEFF
	// -------------------------------------------------------------------------------------------


	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {
		// Przepisanie danych z EDP_MASTER do obrazu robota

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		// Wyznaczenie liczby przedzialow interpolacji oraz pozostalych parametrow ruchu

		number_of_intervals = (int) ceil(tip.motion_time/INTERVAL);

		switch (tip.arm_type) {
			case MOTOR:
				the_robot->EDP_data.set_arm_type = MOTOR;
				break;

			case JOINT:
				the_robot->EDP_data.set_arm_type = JOINT;
				break;

			case XYZ_EULER_ZYZ:
				the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
				break;

			case XYZ_ANGLE_AXIS:
				the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
				break;

			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch


		the_robot->EDP_data.instruction_type = SET;
		the_robot->EDP_data.set_type = ARM_DV; // ARM
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.next_interpolation_type = MIM;
		the_robot->EDP_data.motion_steps = (WORD) (INTERVAL/STEP);
		the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-2;

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------


	if (node_counter <= t(j+1, 1)) {

		double A;
		double B;
		double C;
		double D;
		// Tablice do przechowywania punktowych wartosci przyspieszen i predkosci
		double acc[6];
		double vel[6];

		switch (tip.arm_type) {
			// ---------------------------------------------------------------------------------------------------------
			case MOTOR:
				the_robot->EDP_data.set_arm_type = MOTOR;

				for (i = 0; i <MAX_SERVOS_NR; i++) {
					A = (t(j+1, i) - node_counter) / (t(j+1, i) - t(j, i));
B					= 1-A;
					C = (1.0/6.0)*(A*A*A-A)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );
					D = (1.0/6.0)*(B*B*B-B)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );

					acc[i] = A*a(j, i) + B*a(j+1, i);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if

					vel[i] = ( (y(j+1, i) - y(j, i)) / (t(j+1, i) - t(j, i)) ) - ((3.0*A*A - 1.0)/6.0 ) * (t(j+1, i)
							- t(j, i)) * a(j, i) + ((3.0*B*B - 1.0)/6.0 ) * (t(j+1, i) - t(j, i)) * a(j+1, i) ;
					if (fabs(vel[i] * (1.0/STEP)) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if

					the_robot->EDP_data.next_motor_arm_coordinates[i] = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D
							*a(j+1, i);

				} // end:for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case JOINT:
				the_robot->EDP_data.set_arm_type = JOINT;

				for (i = 0; i <MAX_SERVOS_NR; i++) {
					A = (t(j+1, i) - node_counter) / (t(j+1, i) - t(j, i));
B					= 1-A;
					C = (1.0/6.0)*(A*A*A-A)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );
					D = (1.0/6.0)*(B*B*B-B)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );

					acc[i] = A*a(j, i) + B*a(j+1, i);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if

					vel[i] = ( (y(j+1, i) - y(j, i)) / (t(j+1, i) - t(j, i)) ) - ((3.0*A*A - 1.0)/6.0 ) * (t(j+1, i)
							- t(j, i)) * a(j, i) + ((3.0*B*B - 1.0)/6.0 ) * (t(j+1, i) - t(j, i)) * a(j+1, i) ;
					if (fabs(vel[i] * (1.0/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if

					the_robot->EDP_data.next_joint_arm_coordinates[i] = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D
							*a(j+1, i) ;

				} // end:for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case XYZ_EULER_ZYZ:
				the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;

				for (i = 0; i < 7; i++) {
					A = (t(j+1, i) - node_counter) / (t(j+1, i) - t(j, i));
B					= 1-A;
					C = (1.0/6.0)*(A*A*A-A)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );
					D = (1.0/6.0)*(B*B*B-B)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );

					acc[i] = A*a(j, i) + B*a(j+1, i);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if

					vel[i] = ( (y(j+1, i) - y(j, i)) / (t(j+1, i) - t(j, i)) ) - ((3.0*A*A - 1.0)/6.0 ) * (t(j+1, i)
							- t(j, i)) * a(j, i) + ((3.0*B*B - 1.0)/6.0 ) * (t(j+1, i) - t(j, i)) * a(j+1, i) ;
					if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->EDP_data.next_gripper_coordinate = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D*a(j+1, i) ;
					} else {
						the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D
								*a(j+1, i) ;
					}
				} // end:for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case XYZ_ANGLE_AXIS:
				the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;

				for (i = 0; i < 6; i++) {
					A = (t(j+1, i) - node_counter) / (t(j+1, i) - t(j, i));
B					= 1-A;
					C = (1.0/6.0)*(A*A*A-A)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );
					D = (1.0/6.0)*(B*B*B-B)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );

					acc[i] = A*a(j, i) + B*a(j+1, i);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if

					vel[i] = ( (y(j+1, i) - y(j, i)) / (t(j+1, i) - t(j, i)) ) - ((3.0*A*A - 1.0)/6.0 ) * (t(j+1, i)
							- t(j, i)) * a(j, i) + ((3.0*B*B - 1.0)/6.0 ) * (t(j+1, i) - t(j, i)) * a(j+1, i) ;
					if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->EDP_data.next_gripper_coordinate = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D*a(j+1, i) ;
					} else {
						the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D*a(j
								+1, i) ;
					}
				} // end:for
				break;
			default:
				break;
				// ---------------------------------------------------------------------------------------------------------
		} // end : switch

	} // end : if (node_counter <= number_of_intervals)

	// ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
	// Czy skonczono interpolacje oraz zniwelowano blad interpolacji?


	if (node_counter <= t(j+1, 1) ) {
		// Ruch do kolejnej pozycji na liscie pozycji nauczonych nie zostal
		// zakonczony - informacja dla Move()
		//    the_robot->create_command (); // Przygotowanie rozkazu dla EDP
		// Rozkaz zostanie wykonany przez Move i odpowiednie dane
		// zostana zaktualizowane w robocie
		return true;
	} else {
		// Po przejsciu wszystkich wezlow interpolacji nalezy przestawic wskaznik listy pozycji
		// nauczonych na nastepna pozycje
		next_pose_list_ptr();
		j++;
		// Ruch do kolejnej pozycji na liscie zakonczono - informacja dla Move()
		return false;
	}

}
; // end :	 ecp_smooth_cubic_spline_generator::next_step

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 5 stopnia,
// z rozpedzaniem i hamowaniem miedzy pozycjami
// ####################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ----------------------------------------------------
// ----------------------------------------------------------------------------------------------

ecp_quintic_spline_generator::ecp_quintic_spline_generator(ecp_task& _ecp_task, double interval = 0.02) :
	ecp_spline_generator(_ecp_task)
{
	INTERVAL = interval; // Dlugosc okresu interpolacji w [sek]
	int i;

	for (i = 0; i <MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 7.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 1.5;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}

	for (i = 0; i < 3; i++) {
		a_max_zyz[i] = 5.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 5.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 5.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	// 	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_quintic_spline_generator");
	v_max_zyz[6] = 5.0; // predkosc katowa 1rad/s
	// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	a_max_zyz[6] = 0.0;
	v_max_aa[6] = 5.0; // predkosc katowa 1rad/s
	// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	a_max_aa[6] = 0.0;
}
; // end : ecp_quintic_spline_generator::ecp_quintic_spline_generator

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_quintic_spline_generator::first_step()
{

	// Poniewaz ten generator wykonuje ruch tylko do kolejnej pozycji na liscie,
	// nie informuje MP o skonczeniu sie listy.

	// Zlecenie odczytu aktualnego polozenia ramienia

	if (!is_pose_list_element())
		return false;
	// Pobranie kolejnej pozycji z listy i wstawienie danych do generatora
	// Na podstawie odczytanej aktualnej pozycji oraz kolejnej pozycji na liscie
	// wyznaczane beda przedzialy interpolacji, czyli makrokroki do realizacji przez EDP.
	get_pose(tip);
	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametry ruchu
	first_interval = true;
	switch (tip.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = MOTOR;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = JOINT;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;

}
; // end : ecp_quintic_spline_generator::first_step()

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool ecp_quintic_spline_generator::next_step()
{

	char messg[128]; // komunikat do SR
	int i; // Licznik

	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		// Wyznaczenie liczby przedzialow interpolacji oraz pozostalych parametrow ruchu
		number_of_intervals = (int) ceil(tip.motion_time/INTERVAL);

		switch (tip.arm_type) {
			case MOTOR:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					the_robot->EDP_data.set_arm_type = MOTOR;
					A0[i]= the_robot->EDP_data.current_motor_arm_coordinates[i];
					A3[i]=(10.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
					A4[i]=(-15.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
					A5[i]=(6.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
									*number_of_intervals);
				} // end:for
				break;

			case JOINT:
				the_robot->EDP_data.set_arm_type = JOINT;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->EDP_data.current_joint_arm_coordinates[i];
					A3[i]=(10.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
					A4[i]=(-15.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
					A5[i]=(6.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
									*number_of_intervals);
				} // end:for
				break;

			case XYZ_EULER_ZYZ:
				the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->EDP_data.current_XYZ_ZYZ_arm_coordinates[i];
					A3[i]=(10.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
					A4[i]=(-15.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
					A5[i]=(6.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
									*number_of_intervals);
				} // end:for
				A0[6]= the_robot->EDP_data.current_gripper_coordinate;
				A3[6]=(10.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals);
				A4[6]=(-15.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
				A5[6]=(6.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
								*number_of_intervals);

				break;
			case XYZ_ANGLE_AXIS:
				the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
					A3[i]=(10.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
					A4[i]=(-15.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
					A5[i]=(6.0*(tip.coordinates[i] - the_robot->EDP_data.current_motor_arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
									*number_of_intervals);
				} // end:for
				A0[6]= the_robot->EDP_data.current_gripper_coordinate;
				A3[6]=(10.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals);
				A4[6]=(-15.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
				A5[6]=(6.0*(tip.coordinates[6] - the_robot->EDP_data.current_gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
								*number_of_intervals);
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		the_robot->EDP_data.instruction_type = SET;
		the_robot->EDP_data.set_type = ARM_DV; // ARM
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.next_interpolation_type = MIM;
		the_robot->EDP_data.motion_steps = (WORD) (INTERVAL/STEP);
		the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-2;

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------


	// (ten, do ktorego zmierza ramie)

	// ---------------------------------  czy obliczono polozenie dla wszystkich wezlow?    ---------------------------------------

	if (node_counter <= number_of_intervals) {
		// Tablice do przechowywania punktowych wartosci przyspieszen i predkosci
		double acc[MAX_SERVOS_NR];
		double vel[MAX_SERVOS_NR];

		switch (tip.arm_type) {
			// ---------------------------------------------------------------------------------------------------------
			case MOTOR:
				for (i = 0; i <MAX_SERVOS_NR; i++) {
					acc[i] = 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
							*(node_counter*node_counter*node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]*(node_counter*node_counter*node_counter)
							+ 5.0*A5[i]*(node_counter*node_counter*node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					the_robot->EDP_data.next_motor_arm_coordinates[i] = A0[i] + A3[i]*(node_counter*node_counter
							*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
							*(node_counter*node_counter*node_counter*node_counter*node_counter);
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case JOINT:
				for (i = 0; i <MAX_SERVOS_NR; i++) {
					acc[i] = 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
							*(node_counter*node_counter*node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]*(node_counter*node_counter*node_counter)
							+ 5.0*A5[i]*(node_counter*node_counter*node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if

					the_robot->EDP_data.next_joint_arm_coordinates[i] = A0[i] + A3[i]*(node_counter*node_counter
							*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
							*(node_counter*node_counter*node_counter*node_counter*node_counter);
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case XYZ_EULER_ZYZ:
				for (i = 0; i < 7; i++) {
					acc[i] = 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
							*(node_counter*node_counter*node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]*(node_counter*node_counter*node_counter)
							+ 5.0*A5[i]*(node_counter*node_counter*node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if

					if (i==6) {
						the_robot->EDP_data.next_gripper_coordinate = A0[i] + A3[i]*(node_counter*node_counter
								*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
								*(node_counter*node_counter*node_counter*node_counter*node_counter);
					} else {
						the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[i] = A0[i] + A3[i]*(node_counter*node_counter
								*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
								*(node_counter*node_counter*node_counter*node_counter*node_counter);
					}

				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case XYZ_ANGLE_AXIS:
				for (i = 0; i < 7; i++) {
					acc[i] = 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
							*(node_counter*node_counter*node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]*(node_counter*node_counter*node_counter)
							+ 5.0*A5[i]*(node_counter*node_counter*node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->EDP_data.next_gripper_coordinate = A0[i] + A3[i]*(node_counter*node_counter
								*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
								*(node_counter*node_counter*node_counter*node_counter*node_counter);
					} else {
						the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = A0[i] + A3[i]*(node_counter*node_counter
								*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
								*(node_counter*node_counter*node_counter*node_counter*node_counter);
					}
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch

	} // end : if (node_counter <= number_of_intervals)
	// -----------------------------------------------------------------------------------------------------------------------------------------


	// Czy skonczono interpolacje oraz zniwelowano blad interpolacji?
	if (node_counter <= number_of_intervals) {
		// Ruch do kolejnej pozycji na liscie pozycji nauczonych nie zostal
		// zakonczony - informacja dla Move()
		//    the_robot->create_command (); // Przygotowanie rozkazu dla EDP
		// Rozkaz zostanie wykonany przez Move i odpowiednie dane
		// zostana zaktualizowane w robocie
		return true;
	} else {
		// Po przejsciu wszystkich wezlow interpolacji nalezy przestawic wskaznik listy pozycji
		// nauczonych na nastepna pozycje
		next_pose_list_ptr();
		// Ruch do kolejnej pozycji na liscie zakonczono - informacja dla Move()
		return false;
	}
}
; // end : ecp_quintic_spline_generator::next_step()

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, z rozpedzaniem i hamowaniem miedzy pozycjami,
// z dokladna zadana pozycja koncowa
// ####################################################################################################


// ####################################################################################################
// ####################################################################################################
// ####################################################################################################
// ELIPSOID -  METODY

// --------------------------------------------------------------------------
// Konstruktor generatora elipsy

ecp_elipsoid_generator::ecp_elipsoid_generator(ecp_task& _ecp_task) :
	ecp_teach_in_generator(_ecp_task)
{
	INTERVAL = 0.006; // Dlugosc okresu interpolacji w [sek]
	int i;

	for (i = 0; i <MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	for (i = 0; i < 3; i++) {
		a_max_zyz[i] = 5.0; // przyspieszenie liniowe koncowki 0.1G
		a_max_zyz[i+3] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_zyz[i] = 5.0; // predkosc liniowa 1m/s
		v_max_zyz[i+3] = 5.0; // predkosc katowa 1rad/s
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_aa[i] = 0.0;
		a_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_aa[i] = 0.0;
		v_max_aa[i+3] = 0.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	}
	a_max_zyz[6] = 5.0; // przyspieszenie liniowe koncowki 0.1G
	// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
	v_max_zyz[6] = 5.0; // predkosc liniowa 1m/s
	// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	a_max_aa[6] = 0.0;
	// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
	v_max_aa[6] = 0.0;
}
; // end: ecp_elipsoid_generator::ecp_elipsoid_generator()


void ecp_elipsoid_generator::get_sample(one_sample& cp, int sn)
{
	cp.ctime = trj_ptr[sn].ctime;
	memcpy(cp.coordinates, trj_ptr[sn].coordinates, MAX_SERVOS_NR*sizeof(double));
}

void ecp_elipsoid_generator::clear_buffer(void)
{
	delete trj_ptr;
}
;
// --------------------------------------------------------------------------
// Generator do odtwarzajacy nauczone pozycje z dokladna pozycja zadana
// Ruch miedzy kolejnymi pozycjami listy ma trojkatny profil predkosci
// Kontakt z MP nastepuje tylko w momencie osiagniecia kolejnej pozycji na liscie

bool ecp_elipsoid_generator::first_step()
{

	// Wstawienie danych do generatora
	// Na podstawie odczytanej aktualnej pozycji oraz kolejnej pozycji na liscie
	// wyznaczane beda przedzialy interpolacji, czyli makrokroki do realizacji przez EDP.
	tip.motion_time = 100.0; // TK - czas koncowy elipsy
	//     tip.motion_time = 50.0; // TK - czas koncowy dla mlynka
	tip.arm_type = XYZ_EULER_ZYZ;

	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametry ruchu
	first_interval = true;
	switch (tip.arm_type) {
		case MOTOR:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = MOTOR;
			break;
		case JOINT:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = JOINT;
			break;
		case XYZ_EULER_ZYZ:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_EULER_ZYZ;
			break;
		case XYZ_ANGLE_AXIS:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // ARM
			the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
			break;
		default:
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;

}
; // end: bool ecp_elipsoid_generator::first_step ( )
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Generator odtwarzajacy nauczone pozycje z dokladna pozycja zadana
bool ecp_elipsoid_generator::next_step()
{

	if (first_interval) {
		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		// Wyznaczenie liczby przedzialow interpolacji oraz pozostalych parametrow ruchu
		number_of_intervals = (int) ceil(tip.motion_time/INTERVAL);
		if ( (number_of_intervals % 2) == 1)
			number_of_intervals++;

		// Przydzial pamieci na pomiary
		if ( (trj_ptr = new one_sample[number_of_intervals]) != NULL)
			memset(trj_ptr, 0, number_of_intervals*sizeof(one_sample)); // zerowanie
		else
			throw ECP_error (NON_FATAL_ERROR, NOT_ENOUGH_MEMORY);

		half_number_of_intervals = number_of_intervals / 2;
		// Wyznaczenie przyspieszen oraz sprawdzenie,
		// czy nie zostaly przekroczone wartosci maksymalne przyspieszen i predkosci
		switch (tip.arm_type) {
			case MOTOR:
				the_robot->EDP_data.set_arm_type = MOTOR;
				break;
			case JOINT:
				the_robot->EDP_data.set_arm_type = JOINT;
				break;
			case XYZ_EULER_ZYZ:
				the_robot->EDP_data.set_arm_type = XYZ_EULER_ZYZ;
				break;
			case XYZ_ANGLE_AXIS:
				the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
		the_robot->EDP_data.instruction_type = SET_GET;
		the_robot->EDP_data.set_type = ARM_DV; // ARM
		the_robot->EDP_data.get_type = ARM_DV; // ARM
		the_robot->EDP_data.get_arm_type = JOINT;
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.next_interpolation_type = MIM;
		the_robot->EDP_data.motion_steps = (WORD) (INTERVAL/STEP);
		the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps-2;
		first_interval = false;

	}
	trj_ptr[node_counter-1].ctime = (node_counter-1)*INTERVAL;
	memcpy(trj_ptr[node_counter-1].coordinates, the_robot->EDP_data.current_joint_arm_coordinates, MAX_SERVOS_NR*sizeof(double));

	// (ten, do ktorego zmierza ramie)

	if (node_counter < number_of_intervals) {
		// Obliczenie polozenia na podstawie danych interpolacyjnych (zgodnie z uplywem czasu)
		switch (tip.arm_type) {
			case MOTOR:
				break;
			case JOINT:
				break;
			case XYZ_EULER_ZYZ:
				// trajektoria zadana
				// elipsa
				the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] = 0.7 * sin(0.03*(node_counter*INTERVAL-50));
				the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] = 1.5 - 0.4 * cos(0.03*(node_counter*INTERVAL-50));
				the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] = 1;
				the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] = M_PI/2. -3./360.*M_PI * (node_counter*INTERVAL-50);
				the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] = M_PI/2.;
				the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5] = -M_PI;

				// mlynek w osobliwosci
				/*
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] = 0.2 *
				 sin(M_PI/20.*(node_counter*INTERVAL)+1);
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] = 0.9 +0.2 *
				 cos(M_PI/20.*(node_counter*INTERVAL)+1);
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] = 1;
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] =
				 M_PI/2. -0.3*sin(M_PI/20.* (node_counter*INTERVAL)+1);
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] = M_PI/2.;
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5] = -M_PI;
				 */

				// mlynek1
				/* 		the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[0] = 0.7+0.2 *
				 sin(M_PI/20.*(node_counter*INTERVAL));
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[1] = 0.5 +0.2 *
				 cos(M_PI/20.*(node_counter*INTERVAL));
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[2] = 1;
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[3] =
				 0.3*sin(M_PI/20.* (node_counter*INTERVAL));
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[4] = M_PI/2.;
				 the_robot->EDP_data.next_XYZ_ZYZ_arm_coordinates[5] = -M_PI;
				 */

				break;
			case XYZ_ANGLE_AXIS:
				break;
			default:
				throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
	}

	// Czy skonczono interpolacje oraz zniwelowano blad interpolacji?
	if (node_counter < number_of_intervals) {
		// Ruch do kolejnej pozycji na liscie pozycji nauczonych nie zostal
		// zakonczony - informacja dla Move()
		//    the_robot->create_command (); // Przygotowanie rozkazu dla EDP
		// Rozkaz zostanie wykonany przez Move i odpowiednie dane
		// zostana zaktualizowane w robocie
		return true;
	} else {
		// Po przejsciu wszystkich wezlow interpolacji nalezy przestawic wskaznik listy pozycji
		// nauczonych na nastepna pozycje
		//   next_pose_list_ptr ();
		// Ruch do kolejnej pozycji na liscie zakonczono - informacja dla Move()
		return false;
	}
}
; // end: bool ecp_elipsoid_generator::next_step ( )
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Zapis rzeczywistej trajektorii do pliku
void ecp_save_trajectory(ecp_elipsoid_generator& the_generator, ecp_task& _ecp_task)
{
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP

	uint64_t e; // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba pozycji do zapamietania
	uint64_t i, j; // Liczniki petli
	one_sample cp; // Pojedynczy pomiar

	ecp_to_ui_msg.hdr.type=0;

	ecp_to_ui_msg.ecp_message = SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.dat"); // Wzorzec nazwy pliku
	// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(ECP_message), sizeof(UI_reply)) == -1) {
	if (MsgSend(_ecp_task.UI_fd, &ecp_to_ui_msg, sizeof(ECP_message), &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
		e = errno;
		perror("ECP: Send() to UI failed\n");
		_ecp_task.sr_ecp_msg->message(SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ecp_generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	if (ui_to_ecp_rep.reply == QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return;

	if (chdir(ui_to_ecp_rep.path) != 0) {
		perror(ui_to_ecp_rep.path);
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}
	std::ofstream to_file(ui_to_ecp_rep.filename); // otworz plik do zapisu
	e = errno;
	if (!to_file) {
		perror(ui_to_ecp_rep.filename);
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
	} else {
		number_of_poses = the_generator.get_number_of_intervals();
		printf("OK=%lld   fn=%s\n", number_of_poses, ui_to_ecp_rep.filename);
		for (i = 0; i < number_of_poses; i++) {
			the_generator.get_sample(cp, i);
			to_file << cp.ctime << ' ';
			for (j = 0; j <MAX_SERVOS_NR; j++)
				to_file << cp.coordinates[j] << ' ';
			to_file << '\n';
		}
	}
	to_file.close();
	the_generator.clear_buffer();
} // end: irp6_postument_save_trajectory()
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Zapis danych z kalibracji do pliku
void ecp_save_extended_file(ecp_calibration_generator& the_generator, ecp_operator_reaction_condition& the_condition, ecp_task& _ecp_task)
{
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
	ecp_taught_in_pose tip; // Nauczona pozycja
	ecp_taught_in_pose etip; // Odczytana pozycja
	char *cwd; // Wsk. na nazwe biezacego katalogu
	uint64_t e; // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba pozycji z listy pozycji
	uint64_t number_of_sup; // Liczba pozycji z listy odczytow
	uint64_t i, j; // Liczniki petli

	ecp_to_ui_msg.hdr.type=0;

	ecp_to_ui_msg.ecp_message = SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.cdt"); // Wzorzec nazwy pliku
	// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(ECP_message), sizeof(UI_reply)) == -1) {
	if (MsgSend(_ecp_task.UI_fd, &ecp_to_ui_msg, sizeof(ECP_message), &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
		e = errno;
		perror("ECP: Send() to UI failed\n");
		_ecp_task.sr_ecp_msg->message(SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ecp_generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
	}
	if (ui_to_ecp_rep.reply == QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return;

	// Ustawienie sciezki dostepu do pliku
	cwd = getcwd(NULL, 0);
	if (chdir(ui_to_ecp_rep.path) != 0) {
		perror(ui_to_ecp_rep.path);
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}
	std::ofstream to_file(ui_to_ecp_rep.filename); // otworz plik do zapisu
	e = errno;
	if (!to_file) {
		perror(ui_to_ecp_rep.filename);
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
	} else {
		the_generator.initiate_pose_list(); // inicjacja listy nauczonych pozycji
		the_condition.initiate_supplementary_list(); // inicjacja listy odczytanych pozycji
		number_of_sup = the_condition.supplementary_list_length();
		number_of_poses = the_generator.pose_list_length(); // liczba pozycji
		if (number_of_poses != number_of_sup)
			throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_COMPATIBLE_LISTS);

		to_file << number_of_poses << '\n'; // ???
		for (i = 0; i < number_of_poses; i++) {
			to_file << i << ' ';
			the_generator.get_pose(tip);
			for (j = 0; j <MAX_SERVOS_NR; j++)
				to_file << tip.coordinates[j] << ' ';
			to_file << "    ";
			the_condition.get_supplementary(etip);
			for (j = 0; j <MAX_SERVOS_NR; j++)
				to_file << etip.coordinates[j] << ' ';
			to_file << '\n';
			the_generator.next_pose_list_ptr();
			the_condition.next_supplementary_list_ptr();
		} // end: for
		the_generator.initiate_pose_list(); // inicjacja listy nauczonych pozycji
		the_condition.initiate_supplementary_list(); // inicjacja listy odczytanych pozycji
	}
}
; // // end: irp6_postument_save_extended_file()
// --------------------------------------------------------------------------

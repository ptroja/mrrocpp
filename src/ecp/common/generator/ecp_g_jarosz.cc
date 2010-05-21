// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
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

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

#include "lib/srlib.h"
#include "ecp/common/generator/ecp_g_jarosz.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {


// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

// ---------------------------------  KONSTRUKTOR  ----------------------------------------------

linear::linear(common::task::task& _ecp_task) :
	delta(_ecp_task)
{
}

linear::linear(common::task::task& _ecp_task, lib::trajectory_description tr_des, int mp_communication_mode_arg) :
	delta(_ecp_task)
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
	// 	EDP_buffer.sr_ecp_msg.message("Skonstruowano obiekt klasy irp6p_irp6p_linear_generator");
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool linear::first_step()
{

	switch (td.arm_type) {
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end : switch (td.arm_type)

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool linear::next_step()
{
	int i; // licznik kolejnych wspolrzednych wektora [0..5]

	// Kontakt z MP
	if (node_counter == td.interpolation_node_no + 1) { // Koniec odcinka

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

	switch (td.arm_type) {
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + node_counter*td.coordinate_delta[i]
								/td.interpolation_node_no;
			} // end:for
			break;

		case lib::JOINT:
			for (i=0; i<MAX_SERVOS_NR; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + node_counter*td.coordinate_delta[i]
								/td.interpolation_node_no;
			} // end:for
			break;
/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<6; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + node_counter*td.coordinate_delta[i]
								/td.interpolation_node_no;
			} // end:for
			the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate + node_counter
					*td.coordinate_delta[6]/td.interpolation_node_no;
			break;

		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<6; i++) {
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + node_counter*td.coordinate_delta[i]
								/td.interpolation_node_no;
			} // end:for
			the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate + node_counter
					*td.coordinate_delta[6]/td.interpolation_node_no;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);

	}// end:switch


	return true;

}

// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji
// Iinterpolacja funckja liniowa z parabolicznymin odcinkami krzykowliniowymi
// ####################################################################################################

// ---------------------------------  KONSTRUKTOR  ----------------------------------------------

linear_parabolic::linear_parabolic(common::task::task& _ecp_task, lib::trajectory_description tr_des, const double *time_a, const double *time_b) :
	delta(_ecp_task)
{
	td = tr_des;

	for (int i=0; i<MAX_SERVOS_NR; i++) {
		ta[i]=time_a[i];
		if ( (ta[i]=time_a[i]) <=0 || ta[i] > 1)
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_TIME_SPECIFICATION);
	}

	for (int i=0; i<MAX_SERVOS_NR; i++) {
		tb[i]=time_b[i];
		if ( (tb[i]=time_b[i]) <=0 || tb[i] > 1)
			throw ECP_error(lib::NON_FATAL_ERROR, INVALID_TIME_SPECIFICATION);
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

	// 	EDP_buffer.sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_linear_parabolic_generator");
}

// ----------------------------------------------------------------------------------------------
// --------------------------------  funckja liczaca droge  -------------------------------------
// ----------------------------------------------------------------------------------------------

double linear_parabolic::calculate_s(const double t, const double ta, const double tb)
{
	double s=0;

	if (t <0 || t >1 ){
		throw ECP_error(lib::NON_FATAL_ERROR, INVALID_TIME_SPECIFICATION);
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
	s= s / (0.5*ta*ta+ta*(tb-ta)+0.5*(ta*(1-tb)) ); // przyrost/calka z predkosci

	return s;

}

// ----------------------------------------------------------------------------------------------
// --------------------------------- metoda	first_step -----------------------------------------
// ----------------------------------------------------------------------------------------------

bool linear_parabolic::first_step()
{

	switch (td.arm_type) {

		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end : switch ( td.arm_type )

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool linear_parabolic::next_step()
{
	int i; // licznik kolejnych wspolrzednych wektora [0..5]

	char messg[128]; // komunikat do SR

	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		switch (td.arm_type) {
			case lib::MOTOR:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					prev_s[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				break;

			case lib::JOINT:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					prev_s[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				break;
/*
			case lib::XYZ_EULER_ZYZ:
				for (int i=0; i<6; i++) {
					prev_s[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				prev_s[7] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
				break;
			case lib::XYZ_ANGLE_AXIS:
				for (int i=0; i<6; i++) {
					prev_s[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				prev_s[7] = the_robot->reply_package.arm.pf_def.gripper_coordinate;
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end:switch

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------


	// Kontakt z MP
	if (node_counter-1 == td.interpolation_node_no) { // Koniec odcinka

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

	double acc[MAX_SERVOS_NR];
	double vel[MAX_SERVOS_NR];
	double vel_avg[MAX_SERVOS_NR];

	switch (td.arm_type) {
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				double s = calculate_s((double)node_counter/td.interpolation_node_no, ta[i], tb[i]);
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + s*td.coordinate_delta[i];
				vel_avg[i] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] - prev_s[i];
				acc[i] = vel_avg[i] - prev_vel_avg[i];
				vel[i] = acc[i] * node_counter;

				/* if ( fabs(acc[i]*(1/STEP*STEP) ) > a_max_motor[i] )
				 { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
				 sprintf(messg,"Acceleration in axis %d is %f, max. acc = %f",i, fabs(acc[i]), a_max_motor[i]);
				 sr_ecp_msg.message(messg);
				 throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				 } // end : if
				 if ( fabs(vel[i] * (1/STEP) ) > v_max_motor[i] )
				 { // Sprawdzenie przekroczenia dopuszczalnego predkosci
				 sprintf(messg,"Velocity in axis %d is %f, max. vel = %f",i, fabs(vel[i]) , v_max_motor[i]);
				 sr_ecp_msg.message(messg);
				 throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				 } */// end : if

				prev_s[i] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i];
				prev_vel_avg[i] = vel_avg[i];
			} // end:for
			break;

		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				double s = calculate_s((double)node_counter/td.interpolation_node_no, ta[i], tb[i]);
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
						= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + s*td.coordinate_delta[i];
				vel_avg[i] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] - prev_s[i];
				acc[i] = vel_avg[i] - prev_vel_avg[i];
				vel[i] = acc[i] * node_counter;

				if (fabs(acc[i]*(1/STEP*STEP)) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				if (fabs(vel[i] * (1/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				prev_s[i] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i];
				prev_vel_avg[i] = vel_avg[i];
			} // end:for
			break;
/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<7; i++) {
				double s = calculate_s((double)node_counter/td.interpolation_node_no, ta[i], tb[i]);

				if (i==6) {
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate + s
							*td.coordinate_delta[i];
					vel_avg[i] = the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate - prev_s[i];
				} else {
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
							= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + s*td.coordinate_delta[i];
					vel_avg[i] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] - prev_s[i];
				}
				acc[i] = vel_avg[i] - prev_vel_avg[i];
				vel[i] = acc[i] * node_counter;

				if (fabs(acc[i]*(1/STEP*STEP)) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				if (fabs(vel[i] * (1/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if

				if (i==6) {
					prev_s[i] = the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate;
				} else {
					prev_s[i] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i];
				}

				prev_vel_avg[i] = vel_avg[i];
			} // end:for
			break;

		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<7; i++) {
				double s = calculate_s((double)node_counter/td.interpolation_node_no, ta[i], tb[i]);

				if (i==6) {
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate + s
							*td.coordinate_delta[i];
					vel_avg[i] = the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate - prev_s[i];
				} else {
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
							= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + s*td.coordinate_delta[i];
					vel_avg[i] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] - prev_s[i];
				}

				acc[i] = vel_avg[i] - prev_vel_avg[i];
				vel[i] = acc[i] * node_counter;

				if (fabs(acc[i]*(1/STEP*STEP)) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				if (fabs(vel[i] * (1/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					prev_s[i] = the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate;
				} else {
					prev_s[i] = the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i];
				}
				prev_vel_avg[i] = vel_avg[i];
			} // end:for
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch


	return true;
}

// ####################################################################################################
// Klasa bazowa dla generatorow o zadany przyrost polozenia/orientacji
// wykorzystujacych do interpolacji wielomiany
// ####################################################################################################


polynomial::polynomial(common::task::task& _ecp_task) :
	delta(_ecp_task)
{
}

// ----------------------------------------------------------------------------------------------
// --------------------------------- metoda	first_step -----------------------------------------
// ----------------------------------------------------------------------------------------------

bool polynomial::first_step()
{

	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametr A0 wielomianu, kt�ry wymaga znajomo�ci pozycji aktualnej ramienia
	first_interval = true;

	switch (td.arm_type) {
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;

		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end : switch ( td.arm_type )

	return true;
}

// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji wykorzystuj�cy do interpolacji wielomian 3 stopnia
// ciaglosc predkosci
// predkosc poczatkowa i koncowa moze byc zadawana
// ####################################################################################################


// ----------------------------------------------------------------------------------------------
// -----------------  konstruktor dla dla zadanych predkosci vp i vk ----------------------------
// ----------------------------------------------------------------------------------------------

cubic::cubic(common::task::task& _ecp_task, lib::trajectory_description tr_des, double *vp, double *vk) :
	polynomial(_ecp_task)
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

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool cubic::next_step()
{
	int i; // licznik kolejnych wsp�lrzednych wektora [0..MAX_SERVOS_NR]

	char messg[128]; // komunikat do SR


	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {
		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		switch (td.arm_type) {
			case lib::MOTOR:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				break;

			case lib::JOINT:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				break;
/*
			case lib::XYZ_EULER_ZYZ:
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				A0[6]= the_robot->reply_package.arm.pf_def.gripper_coordinate;
				break;
			case lib::XYZ_ANGLE_AXIS:
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				A0[6]= the_robot->reply_package.arm.pf_def.gripper_coordinate;
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------

	// Kontakt z MP
	if (node_counter-1 == td.interpolation_node_no) { // Koniec odcinka
		//     ecp_t.set_ecp_reply (lib::TASK_TERMINATED); // by Y

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

	double acc[MAX_SERVOS_NR];
	double vel[MAX_SERVOS_NR];

	switch (td.arm_type) {
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				acc[i] = ( 2.0*A2[i] + 6.0*A3[i]*(node_counter) ) * ( 1.0
						/ (STEP*the_robot->ecp_command.instruction.motion_steps*STEP*the_robot->ecp_command.instruction.motion_steps));

				if (fabs(acc[i]) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if

				vel[i] = (A1[i] + 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) ) * ( 1.0
						/ (STEP*the_robot->ecp_command.instruction.motion_steps));

				if (fabs(vel[i]) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if

				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
						*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
			} // end:for
			break;

		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<MAX_SERVOS_NR; i++) {
				acc[i] = ( 2.0*A2[i] + 6.0*A3[i]*(node_counter) ) * ( 1.0 / (STEP*(the_robot->ecp_command.instruction.motion_steps)*STEP*(the_robot->ecp_command.instruction.motion_steps)));

				if (fabs(acc[i]) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if

				vel[i] =(A1[i] + 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) ) * ( 1.0
						/ (STEP*the_robot->ecp_command.instruction.motion_steps));

				if (fabs(vel[i]) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if

				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
						*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
			} // end:for
			break;
/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<7; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
							*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
				} else {
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]
							*(node_counter*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
				}

			} // end:for
			break;

		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (i=0; i<6; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
							*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
				} else {
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]
							*(node_counter*node_counter) + A3[i]*(node_counter*node_counter*node_counter);
				}
			} // end:for
			break;
*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch

	// skopiowac przygotowany rozkaz dla EDP do bufora wysylkowego


	return true;
}

// ####################################################################################################
// Generator o zadany przyrost polozenia/orientacji wykorzystujacy do interpolacji wielomian 5 stopnia
// ciaglosc predkosci i przyspieszenia
// predkosc/przyspieszenie poczatkowa/e i koncowa/e moze byc zadawana/e
// ####################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ------------------------------------------------------
// ----------------------------------------------------------------------------------------------

quintic::quintic(common::task::task& _ecp_task, lib::trajectory_description tr_des, double *vp, double *vk, double *ap, double *ak) :
	polynomial(_ecp_task)
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

// ----------------------------------------------------------------------------------------------
// ----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool quintic::next_step()
{

	char messg[128]; // komunikat do SR


	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		switch (td.arm_type) {
			case lib::MOTOR:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				break;

			case lib::JOINT:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				break;
/*
			case lib::XYZ_EULER_ZYZ:
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				A0[6]= the_robot->reply_package.arm.pf_def.gripper_coordinate;
				break;
			case lib::XYZ_ANGLE_AXIS:
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
				} // end:for
				A0[6]= the_robot->reply_package.arm.pf_def.gripper_coordinate;
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------

	// Kontakt z MP
	if (node_counter-1 == td.interpolation_node_no) { // Koniec odcinka

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

	double acc[MAX_SERVOS_NR];
	double vel[MAX_SERVOS_NR];

	switch (td.arm_type) {
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (int i=0; i<MAX_SERVOS_NR; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
						*(node_counter*node_counter*node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]
						*(node_counter*node_counter*node_counter) + 5.0*A5[i]*(node_counter*node_counter*node_counter
						*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
						*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]*(node_counter
						*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter*node_counter
						*node_counter*node_counter);
			} // end:for
			break;

		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (int i=0; i<MAX_SERVOS_NR; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
						*(node_counter*node_counter*node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]
						*(node_counter*node_counter*node_counter) + 5.0*A5[i]*(node_counter*node_counter*node_counter
						*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
						*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]*(node_counter
						*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter*node_counter
						*node_counter*node_counter);
			} // end:for
			break;
/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (int i=0; i<6; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
						*(node_counter*node_counter*node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]
						*(node_counter*node_counter*node_counter) + 5.0*A5[i]*(node_counter*node_counter*node_counter
						*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
							*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]*(node_counter
							*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter*node_counter
							*node_counter*node_counter);
				} else {
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]
							*(node_counter*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]
							*(node_counter*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter
							*node_counter*node_counter*node_counter);
				}

			} // end:for
			break;

		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::SET;
			the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			for (int i=0; i<6; i++) {
				acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
						*(node_counter*node_counter*node_counter);
				if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
					sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
				} // end : if
				vel[i] = A1[i] + 2*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]
						*(node_counter*node_counter*node_counter) + 5.0*A5[i]*(node_counter*node_counter*node_counter
						*node_counter);
				if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
					sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
					sr_ecp_msg.message(messg);
					throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
				} // end : if
				if (i==6) {
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A0[i] + A1[i]*(node_counter) + A2[i]*(node_counter
							*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]*(node_counter
							*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter*node_counter
							*node_counter*node_counter);
				} else {
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A1[i]*(node_counter) + A2[i]
							*(node_counter*node_counter) + A3[i]*(node_counter*node_counter*node_counter) + A4[i]
							*(node_counter*node_counter*node_counter*node_counter) + A5[i]*(node_counter*node_counter
							*node_counter*node_counter*node_counter);
				}

			} // end:for
			break;
*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}// end:switch

	// skopiowac przygotowany rozkaz dla EDP do bufora wysylkowego


	return true;
}

// ####################################################################################################
// ############################     Odtwarzanie listy pozycji    ######################################
// ####################################################################################################


spline::spline(common::task::task& _ecp_task) :
	teach_in(_ecp_task)
{
}

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, z rozpedzaniem i hamowaniem miedzy pozycjami,
// z dokladna zadana pozycja koncowa
// ####################################################################################################


// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ------------------------------------------------------
// ----------------------------------------------------------------------------------------------

parabolic_teach_in::parabolic_teach_in(common::task::task& _ecp_task, double interval = 0.02) :
	spline(_ecp_task)
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

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool parabolic_teach_in::first_step()
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
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			break;
		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;
}

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool parabolic_teach_in::next_step()
{
	//double Delta; // roznica polozen aktulanego  i zadanego

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
			case lib::MOTOR:
				the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
				for (i=0; i <MAX_SERVOS_NR; i++) {
					a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i])
							/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if ( 0.5*fabs(a[i] * tip.motion_time) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i+1, 0.5 * a[i] * tip.motion_time, v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case lib::JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
				for (i=0; i <MAX_SERVOS_NR; i++) {
					a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i])
							/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if ( 0.5*fabs(a[i] * tip.motion_time) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i+1, 0.5*a[i] * tip.motion_time, v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
				/*
			case lib::XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
				for (i=0; i < 7; i++) {
					if (i==6) {
						Delta = tip.coordinates[i] - the_robot->reply_package.arm.pf_def.gripper_coordinate;
					} else {
						Delta = tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i];
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
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if ( 0.5*fabs(a[i] * tip.motion_time) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5*a[i]
								* tip.motion_time, v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case lib::XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
				for (i=0; i < 7; i++) {
					if (i==6) {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.gripper_coordinate)
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					} else {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i])
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					}

					if (fabs(a[i]) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5* a[i]
								* tip.motion_time, v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if ( 0.5*fabs(a[i] * tip.motion_time) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5*a[i]
								* tip.motion_time, v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
		the_robot->ecp_command.instruction.instruction_type = lib::SET;
		the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		the_robot->ecp_command.instruction.motion_steps = (uint16_t) (INTERVAL/STEP);
		the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps-2;
		first_interval = false;

	}

	// (ten, do ktorego zmierza ramie)

	if (node_counter <= number_of_intervals) {
		// Obliczenie polozenia na podstawie danych interpolacyjnych (zgodnie z uplywem czasu)
		switch (tip.arm_type) {
			case lib::MOTOR:

				for (i = 0; i <MAX_SERVOS_NR; i++) {
					if (node_counter < half_number_of_intervals)
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = tip.coordinates[i] - 0.5 * a[i]
								* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
								- node_counter*INTERVAL);
				}
				break;
			case lib::JOINT:
				for (i = 0; i <MAX_SERVOS_NR; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = tip.coordinates[i] - 0.5 * a[i]
								* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
								- node_counter*INTERVAL);
				break;
				/*
			case lib::XYZ_EULER_ZYZ:
				for (i = 0; i < 6; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = tip.coordinates[i] - 0.5 * a[i]
								* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
								- node_counter*INTERVAL);

				if (node_counter < half_number_of_intervals)
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate + 0.5
							* a[6]* node_counter*INTERVAL*node_counter*INTERVAL;
				else
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = tip.coordinates[6] - 0.5 * a[6]
							* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
							- node_counter*INTERVAL);

				break;
			case lib::XYZ_ANGLE_AXIS:
				for (i = 0; i < 6; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = tip.coordinates[i] - 0.5 * a[i]
								* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
								- node_counter*INTERVAL);

				if (node_counter < half_number_of_intervals)
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate + 0.5
							* a[6]* node_counter*INTERVAL*node_counter*INTERVAL;
				else
					the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = tip.coordinates[6] - 0.5 * a[6]
							* (number_of_intervals*INTERVAL - node_counter*INTERVAL)* (number_of_intervals*INTERVAL
							- node_counter*INTERVAL);

				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
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

// ####################################################################################################
// Generator odtwarzajacy liste nauczonych pozycji, wykorzystywany do kalibracji
// ####################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ------------------------------------------------------
// ----------------------------------------------------------------------------------------------

calibration::calibration(common::task::task& _ecp_task, double interval) :
	spline(_ecp_task)
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



// --------------------------------------------------------------------------
// Zapis danych z kalibracji do pliku
void calibration::ecp_save_extended_file(operator_reaction_condition& the_condition)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP
	ecp_taught_in_pose tip; // Nauczona pozycja
	ecp_taught_in_pose etip; // Odczytana pozycja
	char *cwd; // Wsk. na nazwe biezacego katalogu
	uint64_t e; // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba pozycji z listy pozycji
	uint64_t number_of_sup; // Liczba pozycji z listy odczytow
	uint64_t i, j; // Liczniki petli

	ecp_to_ui_msg.ecp_message = lib::SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.cdt"); // Wzorzec nazwy pliku
	// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type=0;

	if (MsgSend(ecp_t.UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	if (messip::port_send(ecp_t.UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
#endif
		e = errno;
		perror("ECP: Send() to UI failed");
		ecp_t.sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}
	if (ui_to_ecp_rep.reply == lib::QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return;

	// Ustawienie sciezki dostepu do pliku
	cwd = getcwd(NULL, 0);
	if (chdir(ui_to_ecp_rep.path) != 0) {
		perror(ui_to_ecp_rep.path);
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}
	std::ofstream to_file(ui_to_ecp_rep.filename); // otworz plik do zapisu
	e = errno;
	if (!to_file) {
		perror(ui_to_ecp_rep.filename);
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	} else {
		initiate_pose_list(); // inicjacja listy nauczonych pozycji
		the_condition.initiate_supplementary_list(); // inicjacja listy odczytanych pozycji
		number_of_sup = the_condition.supplementary_list_length();
		number_of_poses = pose_list_length(); // liczba pozycji
		if (number_of_poses != number_of_sup)
			throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_COMPATIBLE_LISTS);

		to_file << number_of_poses << '\n'; // ???
		for (i = 0; i < number_of_poses; i++) {
			to_file << i << ' ';
			get_pose(tip);
			for (j = 0; j <MAX_SERVOS_NR; j++)
				to_file << tip.coordinates[j] << ' ';
			to_file << "    ";
			the_condition.get_supplementary(etip);
			for (j = 0; j <MAX_SERVOS_NR; j++)
				to_file << etip.coordinates[j] << ' ';
			to_file << '\n';
			next_pose_list_ptr();
			the_condition.next_supplementary_list_ptr();
		} // end: for
		initiate_pose_list(); // inicjacja listy nauczonych pozycji
		the_condition.initiate_supplementary_list(); // inicjacja listy odczytanych pozycji
	}
}
// --------------------------------------------------------------------------




// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool calibration::first_step()
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
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			break;
		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;

}

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool calibration::next_step()
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
			case lib::MOTOR:
				the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
				for (i=0; i <MAX_SERVOS_NR; i++) {
					a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i])
							/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if (fabs(0.5* a[i] * tip.motion_time) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i+1, 0.5* a[i] * tip.motion_time, v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case lib::JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
				for (i=0; i <MAX_SERVOS_NR; i++) {
					a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i])
							/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					if (fabs(a[i]) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if (fabs(0.5* a[i] * tip.motion_time) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i+1, 0.5* a[i] * tip.motion_time, v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
				/*
			case lib::XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
				for (i=0; i < 7; i++) {
					if (i==6) {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.gripper_coordinate)
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					} else {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i])
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					}

					if (fabs(a[i]) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in coordinate %d is %f, max. acc = %f", i+1, fabs(a[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if (fabs(0.5* a[i] * tip.motion_time) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5* a[i]
								* tip.motion_time, v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
			case lib::XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
				for (i=0; i < 7; i++) {
					if (i==6) {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.gripper_coordinate)
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					} else {
						a[i] = 4.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i])
								/(number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
					}
					if (fabs(a[i]) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5* a[i]
								* tip.motion_time, v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					}
					if (fabs(0.5* a[i] * tip.motion_time) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in coordinate %d is %f, max. vel = %f", i+1, 0.5* a[i]
								* tip.motion_time, v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					}
				}
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
		the_robot->ecp_command.instruction.instruction_type = lib::SET;
		the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		the_robot->ecp_command.instruction.motion_steps = (uint16_t) (INTERVAL/STEP);
		the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps-2;
		first_interval = false;

	}

	// (ten, do ktorego zmierza ramie)

	if (node_counter <= number_of_intervals)
		// Obliczenie polozenia na podstawie danych interpolacyjnych (zgodnie z uplywem czasu)
		switch (tip.arm_type) {
			case lib::MOTOR:
				for (i = 0; i <MAX_SERVOS_NR; i++) {
					if (node_counter < half_number_of_intervals)
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + a[i] * (node_counter*INTERVAL
										*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
										-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
				}
				break;
			case lib::JOINT:
				for (i = 0; i <MAX_SERVOS_NR; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + a[i] * (node_counter*INTERVAL
										*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
										-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);
				break;
				/*
			case lib::XYZ_EULER_ZYZ:
				for (i = 0; i < 6; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + a[i] * (node_counter
										*INTERVAL*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter
										*INTERVAL -0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);

				if (node_counter < half_number_of_intervals)
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[6]
							= the_robot->reply_package.arm.pf_def.arm_coordinates[6] + 0.5* a[6]* node_counter*INTERVAL
									*node_counter*INTERVAL;
				else
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[6]
							= the_robot->reply_package.arm.pf_def.arm_coordinates[6] + a[6] * (node_counter*INTERVAL
									*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
									-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);

				break;
			case lib::XYZ_ANGLE_AXIS:
				for (i = 0; i < 6; i++)
					if (node_counter < half_number_of_intervals)
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + 0.5* a[i]* node_counter
										*INTERVAL*node_counter*INTERVAL;
					else
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i]
								= the_robot->reply_package.arm.pf_def.arm_coordinates[i] + a[i] * (node_counter*INTERVAL
										*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
										-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);

				if (node_counter < half_number_of_intervals)
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[6]
							= the_robot->reply_package.arm.pf_def.arm_coordinates[6] + 0.5* a[6]* node_counter*INTERVAL
									*node_counter*INTERVAL;
				else
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[6]
							= the_robot->reply_package.arm.pf_def.arm_coordinates[6] + a[6] * (node_counter*INTERVAL
									*number_of_intervals*INTERVAL -0.5*node_counter*INTERVAL*node_counter*INTERVAL
									-0.25*number_of_intervals*INTERVAL*number_of_intervals*INTERVAL);

				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
	else { // Zniwelowanie ewentualnego bledu miedzy efektem interpolacji a faktyczna pozycja nauczona - celem ruchu
		the_robot->ecp_command.instruction.motion_steps = 25;
		the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps;
		switch (tip.arm_type) {
			case lib::MOTOR:
				memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case lib::JOINT:
				memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
				/*
			case lib::XYZ_EULER_ZYZ:
				memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case lib::XYZ_ANGLE_AXIS:
				memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
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

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 3 stopnia,
// z rozpedzaniem i hamowaniem miedzy pozycjami
// ####################################################################################################


cubic_spline::cubic_spline(common::task::task& _ecp_task, double interval = 0.02) :
	spline(_ecp_task)
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

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool cubic_spline::first_step()
{

	if (!is_pose_list_element()) {
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
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			break;
		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;
}

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool cubic_spline::next_step()
{

	char messg[128]; // komunikat do SR

	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		// Wyznaczenie liczby przedzialow interpolacji oraz pozostalych parametrow ruchu
		number_of_intervals = (int) ceil(tip.motion_time/INTERVAL);

		switch (tip.arm_type) {
			case lib::MOTOR:
				the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					A2[i]=(3.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals);
					A3[i]=(-2.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
				} // end:for
				break;

			case lib::JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					A2[i]=(3.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals);
					A3[i]=(-2.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
				} // end:for
				break;
/*
			case lib::XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					A2[i]=(3.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals);
					A3[i]=(-2.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
				} // end:for

				A0[6]= the_robot->reply_package.arm.pf_def.gripper_coordinate;
				A2[6]=(3.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals);
				A3[6]=(-2.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals);
				break;
			case lib::XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					A2[i]=(3.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals);
					A3[i]=(-2.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
				} // end:for
				A0[6]= the_robot->reply_package.arm.pf_def.gripper_coordinate;
				A2[6]=(3.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals);
				A3[6]=(-2.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals);
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		the_robot->ecp_command.instruction.instruction_type = lib::SET;
		the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		the_robot->ecp_command.instruction.motion_steps = (uint16_t) (INTERVAL/STEP);
		the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps-2;

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
			case lib::MOTOR:
				for (int i = 0; i <MAX_SERVOS_NR; i++) {
					acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A2[i]*(node_counter*node_counter)
							+ A3[i]*(node_counter*node_counter*node_counter);
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case lib::JOINT:
				for (int i=0; i <MAX_SERVOS_NR; i++) {
					acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A2[i]*(node_counter*node_counter)
							+ A3[i]*(node_counter*node_counter*node_counter);
				} // end : for
				break;
				/*
				// ---------------------------------------------------------------------------------------------------------
			case lib::XYZ_EULER_ZYZ:
				for (int i = 0; i < 7; i++) {
					acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A0[i] + A2[i]*(node_counter*node_counter) + A3[i]
								*(node_counter*node_counter*node_counter);
					} else {
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A2[i]*(node_counter*node_counter)
								+ A3[i]*(node_counter*node_counter*node_counter);
					}
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case lib::XYZ_ANGLE_AXIS:
				for (int i = 0; i < 7; i++) {
					acc[i] = 2.0*A2[i] + 6.0*A3[i]*(node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 2.0*A2[i]*(node_counter) + 3.0*A3[i]*(node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A0[i] + A2[i]*(node_counter*node_counter) + A3[i]
								*(node_counter*node_counter*node_counter);
					} else {
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A2[i]*(node_counter*node_counter)
								+ A3[i]*(node_counter*node_counter*node_counter);
					}
				} // end : for
				break;
				*/
				// ---------------------------------------------------------------------------------------------------------
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
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

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 3 stopnia
// ####################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ----------------------------------------------------
// ----------------------------------------------------------------------------------------------

smooth_cubic_spline::smooth_cubic_spline(common::task::task& _ecp_task, double *vp, double *vk, double interval = 0.02) :
	spline(_ecp_task)
{
	INTERVAL = interval; // Dlugosc okresu interpolacji w [sek]
	build_coeff = true;

	// Tworzymy macierze:
	y.resize(pose_list_length() + 1, 6);
	t.resize(pose_list_length() + 1, 6);
	a.resize(pose_list_length() + 1, 6);

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
	// 	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_smooth_cubic_spline_generator");
	a_max_zyz[6] = 5.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_zyz[6] = 5.0;
	a_max_aa[6] = 0.0; // przyspieszenie katowe koncowki 1rad/s^2
	v_max_aa[6] = 0.0;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------metoda BuildCoeff -------------------------------------------
// ----------------------------------------------------------------------------------------------

void smooth_cubic_spline::Build_Coeff(double *tt, double *yy, int nn, double vvp, double vvk, double *aa)
{
	sr_ecp_msg.message("Metoda Build_Coeff wyznacza przyspieszenia w punktach wezlowych");
	double p, qn, sig, un;

	double uu[nn-2];

	aa[0]= -0.5;
	uu[0]=( 3.0 /(tt[1]-tt[0]) ) * ( (yy[1]-yy[0]) / (tt[1]-tt[0])-vvp);

	for (int i=1; i<=nn-2; i++) { // petla algorytmu trojdiagonalnego

		sig=(tt[i]-tt[i-1])/(tt[i+1]-tt[i-1]);
		p=sig*aa[i-1]+2.0;
		aa[i]=(sig-1.0)/p;
		uu[i]=(yy[i+1]-yy[i])/(tt[i+1]-tt[i]) - (yy[i]-yy[i-1])/(tt[i]-tt[i-1]);
		uu[i]=(6.0*uu[i]/(tt[i+1]-tt[i-1])-sig*uu[i-1])/p;
	}

	qn=0.5;
	un=( 3.0 /(tt[nn-1]-tt[nn-2]) )*(vvk-(yy[nn-1]-yy[nn-2]) / (tt[nn-1]-tt[nn-2]));
	aa[nn-1]=(un-qn*uu[nn-2])/(qn*aa[nn-2]+1.0);

	for (int k=nn-2; k>=0; k--) // petla wstecznego zastepowania algorytmu trojdiagonalnego
	{
		aa[k]=aa[k]*aa[k+1]+uu[k];
	}
}

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool smooth_cubic_spline::first_step()
{

	if (!is_pose_list_element()) {
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
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			break;
		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;
}

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool smooth_cubic_spline::next_step()
{

	char messg[128]; // komunikat do SR

	// ---------------------------------   BUILD COEFF    ---------------------------------------
	if (build_coeff) {

		j=0;

		switch (tip.arm_type) {
			case lib::MOTOR:
				the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					y(0, i) = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					t(0, i) = (double) 0;
				} // end:for
				break;

			case lib::JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					y(0, i) = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					t(0, i) = (double) 0;
				} // end:for
				break;
/*
			case lib::XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
				for (int i=0; i<7; i++) {
					if (i==6) {
						y(0, i) = the_robot->reply_package.arm.pf_def.gripper_coordinate;
					} else {
						y(0, i) = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					}
					t(0, i) = (double) 0;
				} // end:for
				break;

			case lib::XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
				for (int i=0; i<7; i++) {
					if (i==6) {
						y(0, i) = the_robot->reply_package.arm.pf_def.gripper_coordinate;
					} else {
						y(0, i) = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					}
					t(0, i) = (double) 0;
					t(0, i) = (double) 0;
				} // end:for
				break;
*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		// Nastepne wartosci przemieszczen wraz z czasami

		for (j=1; j<=pose_list_length(); j++) {

			switch (tip.arm_type) {
				case lib::MOTOR:
					the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
					for (int i=0; i<MAX_SERVOS_NR; i++) {

						y(j, i) = tip.coordinates[i];
						t(j, i) = (double) ( t(j-1 ,i) + (tip.motion_time/INTERVAL) );
					} // end:for
					break;

				case lib::JOINT:
					the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
					for (int i=0; i<MAX_SERVOS_NR; i++) {
						y(j, i) = tip.coordinates[i];
						t(j, i) = (double) ( t(j-1 ,i) + (tip.motion_time/INTERVAL) );
					} // end:for
					break;
/*
				case lib::XYZ_EULER_ZYZ:
					the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
					for (int i=0; i<7; i++) {
						y(j, i) = tip.coordinates[i];
						t(j, i) = (double) ( t(j-1 ,i) + (tip.motion_time/INTERVAL) );
					} // end:for
					break;

				case lib::XYZ_ANGLE_AXIS:
					the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
					for (int i=0; i<7; i++) {
						y(j, i) = tip.coordinates[i];
						t(j, i) = (double) ( t(j-1 ,i) + (tip.motion_time/INTERVAL) );
					} // end:for
					break;
*/
				default:
					throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
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

			double t_temp[pose_list_length() + 1];
			double y_temp[pose_list_length() + 1];
			double a_temp[pose_list_length() + 1];

			for (int z=0; z<=pose_list_length(); z++) {
				t_temp[z] = t(z, i);
				y_temp[z] = y(z, i);
			}// end:for

			Build_Coeff(t_temp, y_temp, pose_list_length()+1, vp[i], vk[i], a_temp);

			for (int z=0; z<=pose_list_length(); z++) {
				a(z, i)= a_temp[z];
			}// end:for

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
			case lib::MOTOR:
				the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
				break;

			case lib::JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
				break;
/*
			case lib::XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
				break;

			case lib::XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
				break;
*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch


		the_robot->ecp_command.instruction.instruction_type = lib::SET;
		the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		the_robot->ecp_command.instruction.motion_steps = (uint16_t) (INTERVAL/STEP);
		the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps-2;

		first_interval = false;

	} // end:if FIRST INTERVAL
	// -------------------------------------------------------------------------------------------


	if (node_counter <= t(j+1, 1)) {

		double A;
		double B;
		double C;
		double D;
		// Tablice do przechowywania punktowych wartosci przyspieszen i predkosci
		double acc[MAX_SERVOS_NR];
		double vel[MAX_SERVOS_NR];

		switch (tip.arm_type) {
			// ---------------------------------------------------------------------------------------------------------
			case lib::MOTOR:
				the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;

				for (int i = 0; i <MAX_SERVOS_NR; i++) {
					A = (t(j+1, i) - node_counter) / (t(j+1, i) - t(j, i));
					B = 1-A;
					C = (1.0/6.0)*(A*A*A-A)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );
					D = (1.0/6.0)*(B*B*B-B)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );

					acc[i] = A*a(j, i) + B*a(j+1, i);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if

					vel[i] = ( (y(j+1, i) - y(j, i)) / (t(j+1, i) - t(j, i)) ) - ((3.0*A*A - 1.0)/6.0 ) * (t(j+1, i)
							- t(j, i)) * a(j, i) + ((3.0*B*B - 1.0)/6.0 ) * (t(j+1, i) - t(j, i)) * a(j+1, i) ;
					if (fabs(vel[i] * (1.0/STEP)) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if

					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D
							*a(j+1, i);

				} // end:for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case lib::JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;

				for (int i = 0; i <MAX_SERVOS_NR; i++) {
					A = (t(j+1, i) - node_counter) / (t(j+1, i) - t(j, i));
					B = 1-A;
					C = (1.0/6.0)*(A*A*A-A)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );
					D = (1.0/6.0)*(B*B*B-B)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );

					acc[i] = A*a(j, i) + B*a(j+1, i);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if

					vel[i] = ( (y(j+1, i) - y(j, i)) / (t(j+1, i) - t(j, i)) ) - ((3.0*A*A - 1.0)/6.0 ) * (t(j+1, i)
							- t(j, i)) * a(j, i) + ((3.0*B*B - 1.0)/6.0 ) * (t(j+1, i) - t(j, i)) * a(j+1, i) ;
					if (fabs(vel[i] * (1.0/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if

					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D
							*a(j+1, i) ;

				} // end:for
				break;
				/*
				// ---------------------------------------------------------------------------------------------------------
			case lib::XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;

				for (int i = 0; i < 7; i++) {
					A = (t(j+1, i) - node_counter) / (t(j+1, i) - t(j, i));
					B					= 1-A;
					C = (1.0/6.0)*(A*A*A-A)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );
					D = (1.0/6.0)*(B*B*B-B)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );

					acc[i] = A*a(j, i) + B*a(j+1, i);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if

					vel[i] = ( (y(j+1, i) - y(j, i)) / (t(j+1, i) - t(j, i)) ) - ((3.0*A*A - 1.0)/6.0 ) * (t(j+1, i)
							- t(j, i)) * a(j, i) + ((3.0*B*B - 1.0)/6.0 ) * (t(j+1, i) - t(j, i)) * a(j+1, i) ;
					if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D*a(j+1, i) ;
					} else {
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D
								*a(j+1, i) ;
					}
				} // end:for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case lib::XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;

				for (int i = 0; i < 6; i++) {
					A = (t(j+1, i) - node_counter) / (t(j+1, i) - t(j, i));
					B = 1-A;
					C = (1.0/6.0)*(A*A*A-A)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );
					D = (1.0/6.0)*(B*B*B-B)*(t(j+1, i)-t(j, i))*(t(j+1, i)-t(j, i) );

					acc[i] = A*a(j, i) + B*a(j+1, i);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if

					vel[i] = ( (y(j+1, i) - y(j, i)) / (t(j+1, i) - t(j, i)) ) - ((3.0*A*A - 1.0)/6.0 ) * (t(j+1, i)
							- t(j, i)) * a(j, i) + ((3.0*B*B - 1.0)/6.0 ) * (t(j+1, i) - t(j, i)) * a(j+1, i) ;
					if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D*a(j+1, i) ;
					} else {
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A*y(j, i) + B*y(j+1, i) + C*a(j, i) +D*a(j
								+1, i) ;
					}
				} // end:for
				break;
				*/
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

// ####################################################################################################
// Generator interpolujacy sklejanymi wielomianami 5 stopnia,
// z rozpedzaniem i hamowaniem miedzy pozycjami
// ####################################################################################################

// ----------------------------------------------------------------------------------------------
// ---------------------------konstruktor ----------------------------------------------------
// ----------------------------------------------------------------------------------------------

quintic_spline::quintic_spline(common::task::task& _ecp_task, double interval = 0.02) :
	spline(_ecp_task)
{
	INTERVAL = interval; // Dlugosc okresu interpolacji w [sek]

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
	// 	sr_ecp_msg.message("Skonstruowano obiekt klasy ecp_quintic_spline_generator");
	v_max_zyz[6] = 5.0; // predkosc katowa 1rad/s
	// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	a_max_zyz[6] = 0.0;
	v_max_aa[6] = 5.0; // predkosc katowa 1rad/s
	// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	a_max_aa[6] = 0.0;
}

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool quintic_spline::first_step()
{

	// Poniewaz ten generator wykonuje ruch tylko do kolejnej pozycji na liscie,
	// nie informuje MP o skonczeniu sie listy.

	// Zlecenie odczytu aktualnego polozenia ramienia

	if (!is_pose_list_element()) {
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
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			break;
		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;
}

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool quintic_spline::next_step()
{

	char messg[128]; // komunikat do SR

	// ---------------------------------   FIRST INTERVAL    ---------------------------------------
	if (first_interval) {

		// Ponizszych obliczen nie mozna wykonac w first_step, gdyz wtedy odczyt
		// aktualnego polozenia ramienia jeszcze nie zostanie zrealizowany. Zrobi
		// to dopiero execute_motion po wyjsciu z first_step.

		// Wyznaczenie liczby przedzialow interpolacji oraz pozostalych parametrow ruchu
		number_of_intervals = (int) ceil(tip.motion_time/INTERVAL);

		switch (tip.arm_type) {
			case lib::MOTOR:
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					A3[i]=(10.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
					A4[i]=(-15.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
					A5[i]=(6.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
									*number_of_intervals);
				} // end:for
				break;

			case lib::JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
				for (int i=0; i<MAX_SERVOS_NR; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					A3[i]=(10.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
					A4[i]=(-15.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
					A5[i]=(6.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
									*number_of_intervals);
				} // end:for
				break;
/*
			case lib::XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					A3[i]=(10.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
					A4[i]=(-15.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
					A5[i]=(6.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
									*number_of_intervals);
				} // end:for
				A0[6]= the_robot->reply_package.arm.pf_def.gripper_coordinate;
				A3[6]=(10.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals);
				A4[6]=(-15.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
				A5[6]=(6.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
								*number_of_intervals);

				break;
			case lib::XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
				for (int i=0; i<6; i++) {
					A0[i]= the_robot->reply_package.arm.pf_def.arm_coordinates[i];
					A3[i]=(10.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals);
					A4[i]=(-15.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
					A5[i]=(6.0*(tip.coordinates[i] - the_robot->reply_package.arm.pf_def.arm_coordinates[i]) )
							/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
									*number_of_intervals);
				} // end:for
				A0[6]= the_robot->reply_package.arm.pf_def.gripper_coordinate;
				A3[6]=(10.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals);
				A4[6]=(-15.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals*number_of_intervals);
				A5[6]=(6.0*(tip.coordinates[6] - the_robot->reply_package.arm.pf_def.gripper_coordinate) )
						/ (number_of_intervals*number_of_intervals*number_of_intervals *number_of_intervals
								*number_of_intervals);
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		the_robot->ecp_command.instruction.instruction_type = lib::SET;
		the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		the_robot->ecp_command.instruction.motion_steps = (uint16_t) (INTERVAL/STEP);
		the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps-2;

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
			case lib::MOTOR:
				for (int i = 0; i <MAX_SERVOS_NR; i++) {
					acc[i] = 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
							*(node_counter*node_counter*node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]*(node_counter*node_counter*node_counter)
							+ 5.0*A5[i]*(node_counter*node_counter*node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_motor[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_motor[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A3[i]*(node_counter*node_counter
							*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
							*(node_counter*node_counter*node_counter*node_counter*node_counter);
				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case lib::JOINT:
				for (int i = 0; i <MAX_SERVOS_NR; i++) {
					acc[i] = 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
							*(node_counter*node_counter*node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]*(node_counter*node_counter*node_counter)
							+ 5.0*A5[i]*(node_counter*node_counter*node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_joint[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_joint[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if

					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A3[i]*(node_counter*node_counter
							*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
							*(node_counter*node_counter*node_counter*node_counter*node_counter);
				} // end : for
				break;
				/*
				// ---------------------------------------------------------------------------------------------------------
			case lib::XYZ_EULER_ZYZ:
				for (int i = 0; i < 7; i++) {
					acc[i] = 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
							*(node_counter*node_counter*node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]*(node_counter*node_counter*node_counter)
							+ 5.0*A5[i]*(node_counter*node_counter*node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_zyz[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_zyz[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if

					if (i==6) {
						the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A0[i] + A3[i]*(node_counter*node_counter
								*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
								*(node_counter*node_counter*node_counter*node_counter*node_counter);
					} else {
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A3[i]*(node_counter*node_counter
								*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
								*(node_counter*node_counter*node_counter*node_counter*node_counter);
					}

				} // end : for
				break;
				// ---------------------------------------------------------------------------------------------------------
			case lib::XYZ_ANGLE_AXIS:
				for (int i = 0; i < 7; i++) {
					acc[i] = 6.0*A3[i]*(node_counter) + 12.0*A4[i]*(node_counter*node_counter) + 20.0*A5[i]
							*(node_counter*node_counter*node_counter);
					if (fabs(acc[i] * (1.0/(STEP*STEP))) > a_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego przyspieszenia
						sprintf(messg, "Acceleration in axis %d is %f, max. acc = %f", i, fabs(acc[i]), a_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_ACCELERATION_EXCEEDED);
					} // end : if
					vel[i] = 3.0*A3[i]*(node_counter*node_counter) + 4.0*A4[i]*(node_counter*node_counter*node_counter)
							+ 5.0*A5[i]*(node_counter*node_counter*node_counter*node_counter);
					if (fabs(vel[i] * (1.0/STEP)) > v_max_aa[i]) { // Sprawdzenie przekroczenia dopuszczalnego predkosci
						sprintf(messg, "Velocity in axis %d is %f, max. vel = %f", i, fabs(vel[i]) , v_max_aa[i]);
						sr_ecp_msg.message(messg);
						throw ECP_error (lib::NON_FATAL_ERROR, MAX_VELOCITY_EXCEEDED);
					} // end : if
					if (i==6) {
						the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = A0[i] + A3[i]*(node_counter*node_counter
								*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
								*(node_counter*node_counter*node_counter*node_counter*node_counter);
					} else {
						the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = A0[i] + A3[i]*(node_counter*node_counter
								*node_counter) + A4[i]*(node_counter*node_counter*node_counter*node_counter) + A5[i]
								*(node_counter*node_counter*node_counter*node_counter*node_counter);
					}
				} // end : for
				break;
				*/
				// ---------------------------------------------------------------------------------------------------------
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
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

elipsoid::elipsoid(common::task::task& _ecp_task) :
	teach_in(_ecp_task)
{
	INTERVAL = 0.006; // Dlugosc okresu interpolacji w [sek]

	for (int i = 0; i < MAX_SERVOS_NR; i++) {
		a_max_motor[i] = 100.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_motor[i] = 120.0;
		// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
		a_max_joint[i] = 0.0;
		// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
		v_max_joint[i] = 0.0;
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
	a_max_zyz[6] = 5.0; // przyspieszenie liniowe koncowki 0.1G
	// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
	v_max_zyz[6] = 5.0; // predkosc liniowa 1m/s
	// tablica dopuszczalnych predkosci dla kolejnych osi/wspolrzednych
	a_max_aa[6] = 0.0;
	// tablica dopuszczalnych przyspieszen dla kolejnych osi/wspolrzednych
	v_max_aa[6] = 0.0;
}



// --------------------------------------------------------------------------
// Zapis rzeczywistej trajektorii do pliku
void elipsoid::ecp_save_trajectory()
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP

	uint64_t e; // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba pozycji do zapamietania
	uint64_t i, j; // Liczniki petli
	one_sample cp; // Pojedynczy pomiar

	ecp_to_ui_msg.ecp_message = lib::SAVE_FILE; // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.dat"); // Wzorzec nazwy pliku
	// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type=0;

	if (MsgSend(ecp_t.UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	if (messip::port_send(ecp_t.UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
#endif
		e = errno;
		perror("ECP: Send() to UI failed");
		ecp_t.sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw generator::ECP_error(lib::SYSTEM_ERROR, 0);
	}

	if (ui_to_ecp_rep.reply == lib::QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return;

	if (chdir(ui_to_ecp_rep.path) != 0) {
		perror(ui_to_ecp_rep.path);
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}
	std::ofstream to_file(ui_to_ecp_rep.filename); // otworz plik do zapisu
	e = errno;
	if (!to_file.good()) {
		perror(ui_to_ecp_rep.filename);
		throw generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	} else {
		number_of_poses = get_number_of_intervals();
		printf("OK=%lld   fn=%s\n", number_of_poses, ui_to_ecp_rep.filename);
		for (i = 0; i < number_of_poses; i++) {
			get_sample(cp, i);
			to_file << cp.ctime << ' ';
			for (j = 0; j <MAX_SERVOS_NR; j++)
				to_file << cp.coordinates[j] << ' ';
			to_file << '\n';
		}
	}

	clear_buffer();
} // end: irp6_postument_save_trajectory()
// --------------------------------------------------------------------------


void elipsoid::get_sample(one_sample& cp, int sn)
{
	cp.ctime = trj_ptr[sn].ctime;
	memcpy(cp.coordinates, trj_ptr[sn].coordinates, MAX_SERVOS_NR*sizeof(double));
}

void elipsoid::clear_buffer(void)
{
	delete trj_ptr;
}

// --------------------------------------------------------------------------
// Generator do odtwarzajacy nauczone pozycje z dokladna pozycja zadana
// Ruch miedzy kolejnymi pozycjami listy ma trojkatny profil predkosci
// Kontakt z MP nastepuje tylko w momencie osiagniecia kolejnej pozycji na liscie

bool elipsoid::first_step()
{

	// Wstawienie danych do generatora
	// Na podstawie odczytanej aktualnej pozycji oraz kolejnej pozycji na liscie
	// wyznaczane beda przedzialy interpolacji, czyli makrokroki do realizacji przez EDP.
	tip.motion_time = 100.0; // TK - czas koncowy elipsy
	//     tip.motion_time = 50.0; // TK - czas koncowy dla mlynka
	//tip.arm_type = lib::XYZ_EULER_ZYZ;

	// Zaznaczenie, ze bedzie realizowany pierwszy przedzial interpolacji, wiec trzeba
	// wyznaczyc parametry ruchu
	first_interval = true;
	switch (tip.arm_type) {
		case lib::MOTOR:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			break;
		case lib::JOINT:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			break;
			/*
		case lib::XYZ_EULER_ZYZ:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
			break;
		case lib::XYZ_ANGLE_AXIS:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
			the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
			break;
			*/
		default:
			throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch

	return true;
}

// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Generator odtwarzajacy nauczone pozycje z dokladna pozycja zadana
bool elipsoid::next_step()
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
			throw ECP_error (lib::NON_FATAL_ERROR, NOT_ENOUGH_MEMORY);

		half_number_of_intervals = number_of_intervals / 2;
		// Wyznaczenie przyspieszen oraz sprawdzenie,
		// czy nie zostaly przekroczone wartosci maksymalne przyspieszen i predkosci
		switch (tip.arm_type) {
			case lib::MOTOR:
				the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
				break;
			case lib::JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
				break;
				/*
			case lib::XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
				break;
			case lib::XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
		the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
		the_robot->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
		the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
		the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		the_robot->ecp_command.instruction.motion_steps = (uint16_t) (INTERVAL/STEP);
		the_robot->ecp_command.instruction.value_in_step_no = the_robot->ecp_command.instruction.motion_steps-2;
		first_interval = false;

	}
	trj_ptr[node_counter-1].ctime = (node_counter-1)*INTERVAL;
	memcpy(trj_ptr[node_counter-1].coordinates, the_robot->reply_package.arm.pf_def.arm_coordinates, MAX_SERVOS_NR*sizeof(double));

	// (ten, do ktorego zmierza ramie)

	if (node_counter < number_of_intervals) {
		// Obliczenie polozenia na podstawie danych interpolacyjnych (zgodnie z uplywem czasu)
		switch (tip.arm_type) {
			case lib::MOTOR:
				break;
			case lib::JOINT:
				break;
				/*
			case lib::XYZ_EULER_ZYZ:
				// trajektoria zadana
				// elipsa
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = 0.7 * sin(0.03*(node_counter*INTERVAL-50));
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = 1.5 - 0.4 * cos(0.03*(node_counter*INTERVAL-50));
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = 1;
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[3] = M_PI/2. -3./360.*M_PI * (node_counter*INTERVAL-50);
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[4] = M_PI/2.;
				the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[5] = -M_PI;


				break;
			case lib::XYZ_ANGLE_AXIS:
				break;
				*/
			default:
				throw ECP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
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
// --------------------------------------------------------------------------

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

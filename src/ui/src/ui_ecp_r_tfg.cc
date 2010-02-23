// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "ecp/irp6p_tfg/ecp_r_irp6p_tfg.h"
#include "ui/ui_ecp_r_tfg.h"

#include <math.h>

// ---------------------------------------------------------------
ui_tfg_robot::ui_tfg_robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name)
{

	switch (_robot_name)
	{
		case lib::ROBOT_IRP6OT_TFG:
			ecp = new ecp::irp6ot_tfg::robot(_config, _sr_ecp_msg);
			break;
		case lib::ROBOT_IRP6P_TFG:
			ecp = new ecp::irp6p_tfg::robot(_config, _sr_ecp_msg);
			break;
		default:
			fprintf(stderr, "ERROR: unknown robot name in ecp_robot ui_tfg_robot::ui_tfg_robot\n");
			ecp = NULL;
			break;
	}

	assert(ecp);

	// Konstruktor klasy
	ecp->ecp_command.instruction.robot_model.kinematic_model.kinematic_model_no = 0;
	ecp->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.get_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.motion_steps = 0;
	ecp->ecp_command.instruction.value_in_step_no = 0;

	ecp->synchronised = false;

	MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
	JOINT_ANGULAR_STEP = 0.0004; // Przyrost kata obrotu w przegubie obrotowym [rad]
	JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]
	END_EFFECTOR_LINEAR_STEP = 0.00002;// Przyrost wspolrzednej polozenia koncowki [m]
	END_EFFECTOR_ANGULAR_STEP = 0.0002; // Przyrost wspolrzednej orientacji koncowki [rad]


}
// ---------------------------------------------------------------

ui_tfg_robot::~ui_tfg_robot()
{
	delete ecp;
}

// ---------------------------------------------------------------
/* // by Y - zdefiniowane w irp6_on_track_robot - przemyslec czy nie trzeba wstawic warunku na poprawnosc synchronizacji
 void ui_tfg_robot::synchronise ( void ) {
 // Zlecenie synchronizacji robota
 ecp->ecp_command.instruction.instruction_type = lib::SYNCHRO;
 ecp->EDP_buffer.send(EDP_fd);  // Wyslanie zlecenia synchronizacji
 ecp->EDP_buffer.query(EDP_fd); // Odebranie wyniku zlecenia
 if (ecp->reply_package.reply_type == lib::SYNCHRO_OK)
 synchronised = true;
 };// end: ui_tfg_robot::synchronise ()
 */
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// virtual  // by Y - wywalone

void ui_tfg_robot::execute_motion(void)
{

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP


	set_ui_state_notification(UI_N_COMMUNICATION);

	// TODO: in QNX/Photon exceptions are handled at the main loop
	// in GTK exceptions triggered signals cannot be handled in main loop

	ecp->execute_motion();
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_tfg_robot::set_desired_position(double d_position[])
{
	// Przepisanie polozen zadanych do tablicy desired_position[]
	for (int j = 0; j < ecp->number_of_servos; j++)
		desired_position[j] = d_position[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_tfg_robot::get_current_position(double c_position[])
{
	// Pobranie aktualnych polozen
	for (int j = 0; j < ecp->number_of_servos; j++)
		c_position[j] = current_position[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// zlecenie odczytu numeru modelu kinematyki i korektora oraz numerow
// algorytmow serwo i numerow zestawow parametrow algorytmow

void ui_tfg_robot::get_kinematic(uint8_t* kinematic_model_no)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.instruction.get_robot_model_type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL
	execute_motion();

	*kinematic_model_no = ecp->reply_package.robot_model.kinematic_model.kinematic_model_no;
}

void ui_tfg_robot::get_servo_algorithm(uint8_t algorithm_no[], uint8_t parameters_no[])
{

	// Zlecenie odczytu numerow algorytmow i zestawow parametrow
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.instruction.get_robot_model_type = lib::SERVO_ALGORITHM; //
	execute_motion();

	// Przepisanie aktualnych numerow algorytmow i zestawow parametrow
	memcpy(algorithm_no, ecp->reply_package.robot_model.servo_algorithm.servo_algorithm_no, ecp->number_of_servos
			* sizeof(uint8_t));
	memcpy(parameters_no, ecp->reply_package.robot_model.servo_algorithm.servo_parameters_no, ecp->number_of_servos
			* sizeof(uint8_t));
}

// do odczytu stanu poczatkowego robota
void ui_tfg_robot::get_controller_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = CONTROLLER_STATE_DEFINITION;

	execute_motion();

	robot_controller_initial_state_l = ecp->reply_package.controller_state;
	ecp->synchronised = robot_controller_initial_state_l.is_synchronised;
}

// ---------------------------------------------------------------
void ui_tfg_robot::set_kinematic(uint8_t kinematic_model_no)
{
	// zlecenie zapisu numeru modelu kinematyki i korektora oraz numerow
	// algorytmow serwo i numerow zestawow parametrow algorytmow

	// Zlecenie zapisu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::SET;
	ecp->ecp_command.instruction.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.instruction.set_robot_model_type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL
	ecp->ecp_command.instruction.get_robot_model_type = lib::ARM_KINEMATIC_MODEL; // ROBOT_MODEL

	ecp->ecp_command.instruction.robot_model.kinematic_model.kinematic_model_no = kinematic_model_no;

	execute_motion();
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_tfg_robot::set_servo_algorithm(uint8_t algorithm_no[], uint8_t parameters_no[])
{

	// Zlecenie zapisu numerow algorytmow i zestawow parametrow
	// Przepisanie zadanych numerow algorytmow i zestawow parametrow
	memcpy(ecp->ecp_command.instruction.robot_model.servo_algorithm.servo_algorithm_no, algorithm_no, ecp->number_of_servos
			* sizeof(uint8_t));
	memcpy(ecp->ecp_command.instruction.robot_model.servo_algorithm.servo_parameters_no, parameters_no, ecp->number_of_servos
			* sizeof(uint8_t));
	ecp->ecp_command.instruction.instruction_type = lib::SET;
	ecp->ecp_command.instruction.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.instruction.set_robot_model_type = lib::SERVO_ALGORITHM; //
	ecp->ecp_command.instruction.get_robot_model_type = lib::SERVO_ALGORITHM; //
	execute_motion();
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_tfg_robot::move_motors(const double final_position[])
{
	// Zlecenie wykonania makrokroku ruchu zadanego dla walow silnikow
	int nr_of_steps, nr_ang, nr_grip; // Liczba krokow
	double max_inc = 0.0, max_inc_grip = 0.0, temp = 0.0; // Zmienne pomocnicze

	/*
	 if (is_synchronised())
	 printf("zsynchronizowany move motors\n");
	 else
	 printf("niezsynchronizowany move motors\n");
	 */
	if (ecp->is_synchronised()) { // Robot zsynchronizowany
		// Odczyt aktualnego polozenia
		//   	printf("is synchronised przed read motors\n");
		read_motors(current_position);

		for (int j = 0; j < ecp->number_of_servos; j++) {
			temp = fabs(final_position[j] - current_position[j]);
			if (j == (ecp->number_of_servos - 1)) // gripper
				max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
			else
				max_inc = (max_inc > temp) ? max_inc : temp;
		}
		nr_ang = (int) ceil(max_inc / MOTOR_STEP);
		nr_grip = (int) ceil(max_inc_grip / MOTOR_GRIPPER_STEP);
		nr_of_steps = (nr_ang > nr_grip) ? nr_ang : nr_grip;
		//  printf("is synchronised za read motors: nr of steps %d\n", nr_of_steps);
		// Parametry zlecenia ruchu i odczytu polozenia
		ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
		ecp->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	} else {
		// printf("!is_synchronised: %f \n",MOTOR_STEP);
		// Robot niezsynchroniozowany
		for (int j = 0; j < ecp->number_of_servos; j++) {
			temp = fabs(final_position[j]);
			if (j == (ecp->number_of_servos - 1)) // gripper
				max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
			else
				max_inc = (max_inc > temp) ? max_inc : temp;
		}
		nr_ang = (int) ceil(max_inc / MOTOR_STEP);
		nr_grip = (int) ceil(max_inc_grip / MOTOR_GRIPPER_STEP);
		nr_of_steps = (nr_ang > nr_grip) ? nr_ang : nr_grip;

		ecp->ecp_command.instruction.instruction_type = lib::SET;
		ecp->ecp_command.instruction.motion_type = lib::RELATIVE;
		ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	}
	ecp->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.get_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	for (int j = 0; j < ecp->number_of_servos; j++) {
		ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];
		/*
		 printf("ui_ecp_aa: %f, %f, %f, %f, %f, %f, %f, %d\n", final_position[0], final_position[1], final_position[2], final_position[3],
		 final_position[4], final_position[5], final_position[6], ecp->ecp_command.instruction.motion_steps);
		 */
		// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp->ecp_command.instruction.motion_steps, ecp->ecp_command.instruction.value_in_step_no, ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[1]);
	}

	execute_motion();

	if (ecp->is_synchronised())
		for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
			current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_tfg_robot::move_joints(const double final_position[])
{
	// Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych
	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_grip, nr_lin;
	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp = 0.0; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;

	// Odczyt aktualnego polozenia
	read_joints(current_position);

	for (int j = 0; j < ecp->number_of_servos; j++) {
		temp = fabs(final_position[j] - current_position[j]);
		if (ecp->robot_name == lib::ROBOT_IRP6_ON_TRACK && j == 0) // tor
			max_inc_lin = (max_inc_lin > temp) ? max_inc_lin : temp;
		else if (j == ecp->number_of_servos - 1) // gripper
			max_inc_grip = (max_inc_grip > temp) ? max_inc_grip : temp;
		else
			max_inc_ang = (max_inc_ang > temp) ? max_inc_ang : temp;
	}

	nr_ang = (int) ceil(max_inc_ang / JOINT_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / JOINT_LINEAR_STEP);
	nr_grip = (int) ceil(max_inc_grip / JOINT_GRIPPER_STEP);
	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	if (ecp->robot_name == lib::ROBOT_IRP6_ON_TRACK) {
		nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	}
	nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;
	// printf("nr_of_steps: %d, nr_grip: %d\n",nr_of_steps,nr_grip);
	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
	ecp->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.get_arm_type = lib::JOINT;
	ecp->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::JOINT;
	ecp->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	for (int j = 0; j < ecp->number_of_servos; j++) {
		ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[j] = final_position[j];
		ecp->ecp_command.instruction.arm.pf_def.desired_torque[j] = final_position[j];
	}

	execute_motion();

	for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_tfg_robot::read_motors(double current_position[])
{
	// Zlecenie odczytu polozenia

	// printf("poczatek read motors\n");
	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;

	execute_motion();
	// printf("dalej za query read motors\n");
	for (int i = 0; i < ecp->number_of_servos; i++) // Przepisanie aktualnych polozen
		// { // printf("current position: %f\n",ecp->reply_package.arm.pf_def.arm_coordinates[i]);
		current_position[i] = ecp->reply_package.arm.pf_def.arm_coordinates[i];
	// 			    }
	// printf("koniec read motors\n");
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_tfg_robot::read_joints(double current_position[])
{
	// Zlecenie odczytu polozenia

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction.get_arm_type = lib::JOINT;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;

	execute_motion();

	for (int i = 0; i < ecp->number_of_servos; i++) // Przepisanie aktualnych polozen
		current_position[i] = ecp->reply_package.arm.pf_def.arm_coordinates[i];
}
// ---------------------------------------------------------------

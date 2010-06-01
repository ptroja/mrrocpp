// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ui/ui_ecp_r_tfg_and_conv.h"

#include "ecp/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "ecp/irp6p_tfg/ecp_r_irp6p_tfg.h"
#include "ecp/conveyor/ecp_r_conv.h"
#include "ecp/spkm/ecp_r_spkm.h"
#include "ecp/smb/ecp_r_smb.h"
#include "ecp/shead/ecp_r_shead.h"

// ---------------------------------------------------------------
ui_tfg_and_conv_robot::ui_tfg_and_conv_robot(lib::configurator &_config,
		lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name) :
	ui_common_robot(_config, _sr_ecp_msg, _robot_name) {

	switch (_robot_name) {
	case lib::ROBOT_IRP6OT_TFG:
		ecp = new ecp::irp6ot_tfg::robot(_config, _sr_ecp_msg);

		MOTOR_STEP = 0.4; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00001; // Przyrost liniowy w przegubach posuwistych [m]

		break;
	case lib::ROBOT_IRP6P_TFG:
		ecp = new ecp::irp6p_tfg::robot(_config, _sr_ecp_msg);

		MOTOR_STEP = 0.4; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00001; // Przyrost liniowy w przegubach posuwistych [m]

		break;
	case lib::ROBOT_CONVEYOR:
		ecp = new ecp::conveyor::robot(_config, _sr_ecp_msg);

		MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]

		break;
	case lib::ROBOT_SPKM:
		ecp = new ecp::spkm::robot(_config, _sr_ecp_msg);

		MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]

		break;

	case lib::ROBOT_SMB:
		ecp = new ecp::smb::robot(_config, _sr_ecp_msg);

		MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]

		break;
	case lib::ROBOT_SHEAD:
		ecp = new ecp::shead::robot(_config, _sr_ecp_msg);

		MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]

		break;
	default:
		fprintf(
				stderr,
				"ERROR: unknown robot name in ecp_robot ui_tfg_and_conv_robot::ui_tfg_and_conv_robot\n");
		ecp = NULL;
		break;
	}

	assert(ecp);

	// Konstruktor klasy
	ecp->ecp_command.instruction.robot_model.kinematic_model.kinematic_model_no
			= 0;
	ecp->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.get_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.motion_steps = 0;
	ecp->ecp_command.instruction.value_in_step_no = 0;

	ecp->synchronised = false;
}

// ---------------------------------------------------------------
void ui_tfg_and_conv_robot::move_motors(const double final_position[]) {
	// Zlecenie wykonania makrokroku ruchu zadanego dla walow silnikow
	int nr_of_steps; // Liczba krokow
	double max_inc = 0.0, temp = 0.0; // Zmienne pomocnicze

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
			max_inc = (max_inc > temp) ? max_inc : temp;
		}
		nr_of_steps = (int) ceil(max_inc / MOTOR_STEP);

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
			max_inc = (max_inc > temp) ? max_inc : temp;
		}
		nr_of_steps = (int) ceil(max_inc / MOTOR_STEP);

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
	for (int j = 0; j < ecp->number_of_servos; j++)
		ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[j]
				= final_position[j];

	// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp_command.instruction.motion_steps, ecp_command.instruction.value_in_step_no, ecp_command.instruction.arm.pf_def.arm_coordinates[1]);

	execute_motion();

	if (ecp->is_synchronised())
		for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
			current_position[j]
					= ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_tfg_and_conv_robot::move_joints(const double final_position[]) {
	// Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych

	double max_inc_lin = 0.0, temp = 0.0; // Zmienne pomocnicze

	// Odczyt aktualnego polozenia
	read_joints(current_position);

	for (int j = 0; j < ecp->number_of_servos; j++) {
		temp = fabs(final_position[j] - current_position[j]);
		max_inc_lin = (max_inc_lin > temp) ? max_inc_lin : temp;
	}

	// Liczba krokow
	const int nr_of_steps = (int) ceil(max_inc_lin / JOINT_LINEAR_STEP);

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

	// cprintf("NOS=%u\n",ecp_command.instruction.motion_steps);

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	for (int j = 0; j < ecp->number_of_servos; j++)
		ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[j]
				= final_position[j];

	execute_motion();

	for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------



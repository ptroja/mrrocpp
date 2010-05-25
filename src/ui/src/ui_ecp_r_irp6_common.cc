// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <cfloat>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ui/ui_ecp_r_irp6_common.h"

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp/irp6_mechatronika/ecp_r_irp6m.h"
#include "ecp/polycrank/ecp_r_polycrank.h"
#include "ecp/smb/ecp_r_smb.h"
#include "ecp/spkm/ecp_r_spkm.h"

// ---------------------------------------------------------------
ui_irp6_common_robot::ui_irp6_common_robot(lib::configurator &_config,
		lib::sr_ecp &_sr_ecp_msg, lib::robot_name_t _robot_name) :
	ui_common_robot(_config, _sr_ecp_msg, _robot_name) {

	switch (_robot_name) {
	case lib::ROBOT_IRP6OT_M:
		ecp = new ecp::irp6ot_m::robot(_config, _sr_ecp_msg);

		MOTOR_GRIPPER_STEP = DBL_MAX;
		JOINT_GRIPPER_STEP = DBL_MAX;// Przyrost liniowy w chwytaku [m]
		END_EFFECTOR_GRIPPER_STEP = DBL_MAX; // Przyrost wspolrzednej orientacji koncowki [rad]


		break;
	case lib::ROBOT_IRP6P_M:
		ecp = new ecp::irp6p_m::robot(_config, _sr_ecp_msg);

		MOTOR_GRIPPER_STEP = DBL_MAX;
		JOINT_GRIPPER_STEP = DBL_MAX;// Przyrost liniowy w chwytaku [m]
		END_EFFECTOR_GRIPPER_STEP = DBL_MAX; // Przyrost wspolrzednej orientacji koncowki [rad]

		break;
	case lib::ROBOT_IRP6_MECHATRONIKA:
		ecp = new ecp::irp6m::robot(_config, _sr_ecp_msg);
		break;
	case lib::ROBOT_POLYCRANK:
		ecp = new ecp::polycrank::robot(_config, _sr_ecp_msg);
		break;
	default:
		fprintf(
				stderr,
				"ERROR: unknown robot name in ecp_robot ui_irp6_common_robot::ui_irp6_common_robot\n");
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

	MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
	JOINT_ANGULAR_STEP = 0.0004; // Przyrost kata obrotu w przegubie obrotowym [rad]
	JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]
	END_EFFECTOR_LINEAR_STEP = 0.00002;// Przyrost wspolrzednej polozenia koncowki [m]
	END_EFFECTOR_ANGULAR_STEP = 0.0002; // Przyrost wspolrzednej orientacji koncowki [rad]
}

// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
void ui_irp6_common_robot::set_tool_xyz_angle_axis(
		const lib::Xyz_Angle_Axis_vector &tool_vector) {
	ecp->ecp_command.instruction.instruction_type = lib::SET;
	ecp->ecp_command.instruction.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	ecp->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(tool_vector);
	tmp.get_frame_tab(
			ecp->ecp_command.instruction.robot_model.tool_frame_def.tool_frame);

	execute_motion();
}
// ---------------------------------------------------------------


// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
void ui_irp6_common_robot::set_tool_xyz_euler_zyz(
		const lib::Xyz_Euler_Zyz_vector &tool_vector) {
	ecp->ecp_command.instruction.instruction_type = lib::SET;
	ecp->ecp_command.instruction.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	ecp->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_euler_zyz(tool_vector);
	tmp.get_frame_tab(
			ecp->ecp_command.instruction.robot_model.tool_frame_def.tool_frame);

	execute_motion();
}
// ---------------------------------------------------------------


// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
void ui_irp6_common_robot::read_tool_xyz_angle_axis(
		lib::Xyz_Angle_Axis_vector & tool_vector) {
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	ecp->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;

	execute_motion();

	lib::Homog_matrix tmp(
			ecp->reply_package.robot_model.tool_frame_def.tool_frame);
	tmp.get_xyz_angle_axis(tool_vector);
}
// ---------------------------------------------------------------


// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
void ui_irp6_common_robot::read_tool_xyz_euler_zyz(
		lib::Xyz_Euler_Zyz_vector &tool_vector) {
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	ecp->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;

	execute_motion();
	lib::Homog_matrix tmp(
			ecp->reply_package.robot_model.tool_frame_def.tool_frame);

	tmp.get_xyz_euler_zyz(tool_vector);
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void ui_irp6_common_robot::move_motors(const double final_position[]) {
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
		ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[j]
				= final_position[j];
		/*
		 printf("ui_ecp_aa: %f, %f, %f, %f, %f, %f, %f, %d\n", final_position[0], final_position[1], final_position[2], final_position[3],
		 final_position[4], final_position[5], final_position[6], ecp->ecp_command.instruction.motion_steps);
		 */
		// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp->ecp_command.instruction.motion_steps, ecp->ecp_command.instruction.value_in_step_no, ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[1]);
	}

	execute_motion();

	if (ecp->is_synchronised())
		for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
			current_position[j]
					= ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_irp6_common_robot::move_joints(const double final_position[]) {
	// Zlecenie wykonania makrokroku ruchu zadanego dla wspolrzednych wewnetrznych
	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_grip, nr_lin;
	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp = 0.0; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;

	// Odczyt aktualnego polozenia
	read_joints(current_position);

	for (int j = 0; j < ecp->number_of_servos; j++) {
		temp = fabs(final_position[j] - current_position[j]);
		if (ecp->robot_name == lib::ROBOT_IRP6OT_M && j == 0) // tor
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
	if (ecp->robot_name == lib::ROBOT_IRP6OT_M) {
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
		ecp->ecp_command.instruction.arm.pf_def.arm_coordinates[j]
				= final_position[j];
		ecp->ecp_command.instruction.arm.pf_def.desired_torque[j]
				= final_position[j];
	}

	execute_motion();

	for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void ui_irp6_common_robot::move_xyz_euler_zyz(const double final_position[7]) {
	// Zlecenie wykonania makrokroku ruchu zadanego we wspolrzednych
	// zewnetrznych: xyz i katy Euler'a Z-Y-Z

	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_lin, nr_grip;
	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp_lin,
			temp_ang, temp_grip; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;
	// Odczyt aktualnego polozenia we wsp. zewn. xyz i katy Euler'a Z-Y-Z
	read_xyz_euler_zyz(current_position);

	for (int j = 0; j < 3; j++) {
		temp_lin = fabs(final_position[j] - current_position[j]);
		temp_ang = fabs(final_position[j + 3] - current_position[j + 3]);
		max_inc_ang = (max_inc_ang > temp_ang) ? max_inc_ang : temp_ang;
		max_inc_lin = (max_inc_lin > temp_lin) ? max_inc_lin : temp_lin;
	}

	temp_grip = fabs(final_position[6] - current_position[6]);
	max_inc_grip = temp_grip;

	nr_ang = (int) ceil(max_inc_ang / END_EFFECTOR_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / END_EFFECTOR_LINEAR_STEP);
	nr_grip = (int) ceil(max_inc_grip / END_EFFECTOR_GRIPPER_STEP);

	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	// cprintf("eNOS=%u\n",ecp->ecp_command.instruction.motion_steps);
	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector(final_position));
	tmp.get_frame_tab(ecp->ecp_command.instruction.arm.pf_def.arm_frame);

	execute_motion();

	for (int j = 0; j < 6; j++) { // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
	}

}
// ---------------------------------------------------------------


void ui_irp6_common_robot::move_xyz_angle_axis(const double final_position[7]) {
	lib::Xyz_Euler_Zyz_vector aa_eul; // tablica przechowujaca polecenie przetransformowane
	// do formy XYZ_EULER_ZYZ
	/*	double x, y, z, alfa, kx, ky, kz;

	 x = final_position[0];
	 y = final_position[1];
	 z = final_position[2];

	 alfa = sqrt(final_position[3] * final_position[3] + final_position[4] * final_position[4] + final_position[5]
	 * final_position[5]);

	 kx = final_position[3] / alfa;
	 ky = final_position[4] / alfa;
	 kz = final_position[5] / alfa;
	 */
	lib::Homog_matrix A;
	A.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(final_position));
	A.get_xyz_euler_zyz(aa_eul); // zadane polecenie w formie XYZ_EULER_ZYZ

	// Odczyt aktualnego polozenia we wsp. zewn. xyz i katy Euler'a Z-Y-Z
	read_xyz_euler_zyz(current_position);

	// Wyznaczenie liczby krokow

	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_lin, nr_grip;
	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp_lin,
			temp_ang, temp_grip; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;
	for (int i = 0; i < 3; i++) {
		temp_lin = fabs(aa_eul[i] - current_position[i]);
		temp_ang = fabs(aa_eul[i + 3] - current_position[i + 3]);
		if (temp_ang > max_inc_ang)
			max_inc_ang = temp_ang;
		if (temp_lin > max_inc_lin)
			max_inc_lin = temp_lin;
	}

	temp_grip = fabs(final_position[6] - current_position[6]);
	max_inc_grip = temp_grip;

	nr_ang = (int) ceil(max_inc_ang / END_EFFECTOR_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / END_EFFECTOR_LINEAR_STEP);
	nr_grip = (int) ceil(max_inc_grip / END_EFFECTOR_GRIPPER_STEP);

	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;

	// Zadano ruch do aktualnej pozycji
	if (nr_of_steps < 1)
		return;

	ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(final_position));
	tmp.get_frame_tab(ecp->ecp_command.instruction.arm.pf_def.arm_frame);

	execute_motion();

	for (int j = 0; j < 6; j++) { // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
	}

}

void ui_irp6_common_robot::move_xyz_angle_axis_relative(
		const double position_increment[7]) {
	int nr_of_steps; // Liczba krokow
	int nr_ang, nr_lin, nr_grip;

	double max_inc_ang = 0.0, max_inc_lin = 0.0, max_inc_grip = 0.0, temp_lin,
			temp_ang; // Zmienne pomocnicze

	max_inc_ang = max_inc_lin = 0.0;
	for (int i = 0; i < 3; i++) {
		temp_lin = fabs(position_increment[i]);
		temp_ang = fabs(position_increment[i + 3]);
		if (temp_ang > max_inc_ang)
			max_inc_ang = temp_ang;
		if (temp_lin > max_inc_lin)
			max_inc_lin = temp_lin;
	}

	max_inc_grip = fabs(position_increment[6]);

	nr_ang = (int) ceil(max_inc_ang / END_EFFECTOR_ANGULAR_STEP);
	nr_lin = (int) ceil(max_inc_lin / END_EFFECTOR_LINEAR_STEP);
	nr_grip = (int) ceil(max_inc_grip / END_EFFECTOR_GRIPPER_STEP);

	nr_of_steps = (nr_ang > nr_lin) ? nr_ang : nr_lin;
	nr_of_steps = (nr_of_steps > nr_grip) ? nr_of_steps : nr_grip;

	// Zadano ruch do aktualnej pozycji
	if (nr_of_steps < 1)
		return;

	ecp->ecp_command.instruction.instruction_type = lib::SET_GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.motion_type = lib::RELATIVE;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	ecp->ecp_command.instruction.motion_steps = nr_of_steps;
	ecp->ecp_command.instruction.value_in_step_no = nr_of_steps;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position_increment));
	tmp.get_frame_tab(ecp->ecp_command.instruction.arm.pf_def.arm_frame);

	execute_motion();
}

// ---------------------------------------------------------------
void ui_irp6_common_robot::read_xyz_euler_zyz(double current_position[]) {
	// Zlecenie odczytu polozenia

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;

	execute_motion();

	lib::Homog_matrix tmp;
	tmp.set_from_frame_tab(ecp->reply_package.arm.pf_def.arm_frame);
	lib::Xyz_Euler_Zyz_vector tmp_vector;
	tmp.get_xyz_euler_zyz(tmp_vector);
	tmp_vector.to_table(current_position);

}
// ---------------------------------------------------------------


void ui_irp6_common_robot::read_xyz_angle_axis(double current_position[]) {
	// Pobranie aktualnego polozenia ramienia robota

	ecp->ecp_command.instruction.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_arm_type = lib::FRAME;
	ecp->ecp_command.instruction.interpolation_type = lib::MIM;
	execute_motion();

	lib::Homog_matrix tmp;
	tmp.set_from_frame_tab(ecp->reply_package.arm.pf_def.arm_frame);
	lib::Xyz_Angle_Axis_vector tmp_vector;
	tmp.get_xyz_angle_axis(tmp_vector);
	tmp_vector.to_table(current_position);

}

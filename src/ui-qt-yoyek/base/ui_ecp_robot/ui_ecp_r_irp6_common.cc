// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <cfloat>
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <cmath>
#include <fcntl.h>
#include <cerrno>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../interface.h"
#include "ui_ecp_r_irp6_common.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace ui {
namespace irp6 {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
	common::EcpRobot(_interface, _robot_name)
{

	if (_robot_name == lib::irp6ot_m::ROBOT_NAME) {

		ecp = new ecp::irp6ot_m::robot(*(_interface.config), *(_interface.all_ecp_msg));

		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.05;
		}

		JOINT_STEP[0] = 0.00004;
		for (int j = 1; j < ecp->number_of_servos; j++) {
			JOINT_STEP[j] = 0.0004;
		}

		for (int j = 0; j < 3; j++) {
			END_EFFECTOR_STEP[j] = 0.00002;
		}

		for (int j = 3; j < 6; j++) {
			END_EFFECTOR_STEP[j] = 0.0002;
		}

	} else if (_robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp = new ecp::irp6p_m::robot(*(_interface.config), *(_interface.all_ecp_msg));

		for (int j = 0; j < ecp->number_of_servos; j++) {
			MOTOR_STEP[j] = 0.05;
			JOINT_STEP[j] = 0.0004;
		}

		for (int j = 0; j < 3; j++) {
			END_EFFECTOR_STEP[j] = 0.00002;
		}

		for (int j = 3; j < 6; j++) {
			END_EFFECTOR_STEP[j] = 0.0002;
		}

	}

	init();

}

// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
void EcpRobot::set_tool_xyz_angle_axis(const lib::Xyz_Angle_Axis_vector &tool_vector)
{
	ecp->ecp_command.instruction_type = lib::SET;
	ecp->ecp_command.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::TOOL_FRAME;
	ecp->ecp_command.get_robot_model_type = lib::TOOL_FRAME;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(tool_vector);
	tmp.get_frame_tab(ecp->ecp_command.robot_model.tool_frame_def.tool_frame);

	execute_motion();
}
// ---------------------------------------------------------------


// ZADANIE NARZEDZIA
// ---------------------------------------------------------------
void EcpRobot::set_tool_xyz_euler_zyz(const lib::Xyz_Euler_Zyz_vector &tool_vector)
{
	ecp->ecp_command.instruction_type = lib::SET;
	ecp->ecp_command.set_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::TOOL_FRAME;
	ecp->ecp_command.get_robot_model_type = lib::TOOL_FRAME;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_euler_zyz(tool_vector);
	tmp.get_frame_tab(ecp->ecp_command.robot_model.tool_frame_def.tool_frame);

	execute_motion();
}
// ---------------------------------------------------------------


// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
void EcpRobot::read_tool_xyz_angle_axis(lib::Xyz_Angle_Axis_vector & tool_vector)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::TOOL_FRAME;
	ecp->ecp_command.get_robot_model_type = lib::TOOL_FRAME;

	execute_motion();

	lib::Homog_matrix tmp(ecp->reply_package.robot_model.tool_frame_def.tool_frame);
	tmp.get_xyz_angle_axis(tool_vector);
}
// ---------------------------------------------------------------


// ODCZYT NARZEDZIA
// ---------------------------------------------------------------
void EcpRobot::read_tool_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector &tool_vector)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_type = ROBOT_MODEL_DEFINITION; // ROBOT_MODEL
	ecp->ecp_command.robot_model.type = lib::TOOL_FRAME;
	ecp->ecp_command.get_robot_model_type = lib::TOOL_FRAME;

	execute_motion();
	lib::Homog_matrix tmp(ecp->reply_package.robot_model.tool_frame_def.tool_frame);

	tmp.get_xyz_euler_zyz(tool_vector);
}
// ---------------------------------------------------------------


// ---------------------------------------------------------------
void EcpRobot::move_xyz_euler_zyz(const double final_position[7])
{
	// Zlecenie wykonania makrokroku ruchu zadanego we wspolrzednych
	// zewnetrznych: xyz i katy Euler'a Z-Y-Z

	int nr_of_steps = 0, nr_tmp = 0; // Liczba krokow
	double temp = 0.0; // Zmienne pomocnicze

	// Odczyt aktualnego polozenia we wsp. zewn. xyz i katy Euler'a Z-Y-Z
	read_xyz_euler_zyz(current_position);

	for (int j = 0; j < 6; j++) {
		temp = fabs(final_position[j] - current_position[j]);
		nr_tmp = (int) ceil(temp / END_EFFECTOR_STEP[j]);
		nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;
	}

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.instruction_type = lib::SET_GET;
	ecp->ecp_command.get_arm_type = lib::FRAME;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::FRAME;
	ecp->ecp_command.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.interpolation_type = lib::MIM;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	// cprintf("eNOS=%u\n",ecp->ecp_command.motion_steps);
	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector(final_position));
	tmp.get_frame_tab(ecp->ecp_command.arm.pf_def.arm_frame);

	execute_motion();

	for (int j = 0; j < 6; j++) { // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
	}

}
// ---------------------------------------------------------------


void EcpRobot::move_xyz_angle_axis(const double final_position[7])
{
	lib::Xyz_Euler_Zyz_vector aa_eul; // tablica przechowujaca polecenie przetransformowane

	lib::Homog_matrix A;
	A.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(final_position));
	A.get_xyz_euler_zyz(aa_eul); // zadane polecenie w formie XYZ_EULER_ZYZ

	// Odczyt aktualnego polozenia we wsp. zewn. xyz i katy Euler'a Z-Y-Z
	read_xyz_euler_zyz(current_position);

	// Wyznaczenie liczby krokow

	int nr_of_steps = 0, nr_tmp = 0; // Liczba krokow
	double temp = 0.0; // Zmienne pomocnicze


	for (int j = 0; j < 6; j++) {
		temp = fabs(aa_eul[j] - current_position[j]);
		nr_tmp = (int) ceil(temp / END_EFFECTOR_STEP[j]);
		nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;

	}

	// Zadano ruch do aktualnej pozycji
	if (nr_of_steps < 1)
		return;

	ecp->ecp_command.instruction_type = lib::SET_GET;
	ecp->ecp_command.get_arm_type = lib::FRAME;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::FRAME;
	ecp->ecp_command.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.interpolation_type = lib::MIM;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(final_position));
	tmp.get_frame_tab(ecp->ecp_command.arm.pf_def.arm_frame);

	execute_motion();

	for (int j = 0; j < 6; j++) { // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
	}

}

void EcpRobot::move_xyz_angle_axis_relative(const double position_increment[7])
{
	int nr_of_steps = 0, nr_tmp = 0; // Liczba krokow
	double temp = 0.0; // Zmienne pomocnicze


	for (int j = 0; j < 6; j++) {

		temp = fabs(position_increment[j]);
		nr_tmp = (int) ceil(temp / END_EFFECTOR_STEP[j]);
		nr_of_steps = (nr_of_steps > nr_tmp) ? nr_of_steps : nr_tmp;

	}

	// Zadano ruch do aktualnej pozycji
	if (nr_of_steps < 1)
		return;

	ecp->ecp_command.instruction_type = lib::SET_GET;
	ecp->ecp_command.get_arm_type = lib::FRAME;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::FRAME;
	ecp->ecp_command.motion_type = lib::RELATIVE;
	ecp->ecp_command.interpolation_type = lib::MIM;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	lib::Homog_matrix tmp;
	tmp.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(position_increment));
	tmp.get_frame_tab(ecp->ecp_command.arm.pf_def.arm_frame);

	execute_motion();
}

// ---------------------------------------------------------------
void EcpRobot::read_xyz_euler_zyz(double current_position[])
{
	// Zlecenie odczytu polozenia

	// Parametry zlecenia ruchu i odczytu polozenia
	ecp->ecp_command.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_arm_type = lib::FRAME;
	ecp->ecp_command.interpolation_type = lib::MIM;

	execute_motion();

	lib::Homog_matrix tmp;
	tmp.set_from_frame_tab(ecp->reply_package.arm.pf_def.arm_frame);
	lib::Xyz_Euler_Zyz_vector tmp_vector;
	tmp.get_xyz_euler_zyz(tmp_vector);
	tmp_vector.to_table(current_position);

}
// ---------------------------------------------------------------


void EcpRobot::read_xyz_angle_axis(double current_position[])
{
	// Pobranie aktualnego polozenia ramienia robota

	ecp->ecp_command.get_type = ARM_DEFINITION;
	ecp->ecp_command.instruction_type = lib::GET;
	ecp->ecp_command.get_arm_type = lib::FRAME;
	ecp->ecp_command.interpolation_type = lib::MIM;
	execute_motion();

	lib::Homog_matrix tmp;
	tmp.set_from_frame_tab(ecp->reply_package.arm.pf_def.arm_frame);
	lib::Xyz_Angle_Axis_vector tmp_vector;
	tmp.get_xyz_angle_axis(tmp_vector);
	tmp_vector.to_table(current_position);

}

}
} //namespace ui
} //namespace mrrocpp

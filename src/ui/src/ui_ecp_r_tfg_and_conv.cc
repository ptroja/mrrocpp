// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <fcntl.h>
#include <cerrno>
#include <cmath>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "ui/src/ui_class.h"

#include "base/lib/sr/srlib.h"

#include "ui/src/ui_ecp_r_tfg_and_conv.h"

#include "robot/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/ecp_r_irp6p_tfg.h"
#include "robot/sarkofag/ecp_r_sarkofag.h"
#include "robot/conveyor/ecp_r_conv.h"
#include "robot/shead/ecp_r_shead.h"
#include "robot/polycrank/ecp_r_polycrank.h"

namespace mrrocpp {
namespace ui {
namespace tfg_and_conv {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface, lib::robot_name_t _robot_name) :
	common::EcpRobot(_interface, _robot_name)
{

	if (_robot_name == lib::irp6ot_tfg::ROBOT_NAME) {
		ecp = new ecp::irp6ot_tfg::robot(*(_interface.config), *(_interface.all_ecp_msg));

		MOTOR_STEP = 0.4; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00001; // Przyrost liniowy w przegubach posuwistych [m]

	} else if (_robot_name == lib::irp6p_tfg::ROBOT_NAME) {
		ecp = new ecp::irp6p_tfg::robot(*(_interface.config), *(_interface.all_ecp_msg));

		MOTOR_STEP = 0.4; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00001; // Przyrost liniowy w przegubach posuwistych [m]

	} else if (_robot_name == lib::sarkofag::ROBOT_NAME) {
		ecp = new ecp::sarkofag::robot(*(_interface.config), *(_interface.all_ecp_msg));

		MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.001; // Przyrost liniowy w przegubach posuwistych [m]

	} else if (_robot_name == lib::conveyor::ROBOT_NAME) {
		ecp = new ecp::conveyor::robot(*(_interface.config), *(_interface.all_ecp_msg));

		MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]

	} else if (_robot_name == lib::shead::ROBOT_NAME) {
		ecp = new ecp::shead::robot(*(_interface.config), *(_interface.all_ecp_msg));

		MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.00004; // Przyrost liniowy w przegubach posuwistych [m]

	} else if (_robot_name == lib::polycrank::ROBOT_NAME) {
		ecp = new ecp::polycrank::robot(*(_interface.config), *(_interface.all_ecp_msg));

		MOTOR_STEP = 0.1; // Przyrost kata obrotu walu silnika [rad]
		JOINT_LINEAR_STEP = 0.0004; // Eksperymentalnie dobrana wartosc, przy ktorej ruch jest płynny
	}

	assert(ecp);

	// Konstruktor klasy
	ecp->ecp_command.robot_model.kinematic_model.kinematic_model_no = 0;
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::MOTOR;
	ecp->ecp_command.motion_steps = 0;
	ecp->ecp_command.value_in_step_no = 0;

	ecp->synchronised = false;
}

// ---------------------------------------------------------------
void EcpRobot::move_motors(const double final_position[])
{
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
		ecp->ecp_command.instruction_type = lib::SET_GET;
		ecp->ecp_command.motion_type = lib::ABSOLUTE;
		ecp->ecp_command.interpolation_type = lib::MIM;
	} else {
		// printf("!is_synchronised: %f \n",MOTOR_STEP);
		// Robot niezsynchroniozowany
		for (int j = 0; j < ecp->number_of_servos; j++) {
			temp = fabs(final_position[j]);
			max_inc = (max_inc > temp) ? max_inc : temp;
		}
		nr_of_steps = (int) ceil(max_inc / MOTOR_STEP);

		ecp->ecp_command.instruction_type = lib::SET;
		ecp->ecp_command.motion_type = lib::RELATIVE;
		ecp->ecp_command.interpolation_type = lib::MIM;
	}
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.get_arm_type = lib::MOTOR;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::MOTOR;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;
	for (int j = 0; j < ecp->number_of_servos; j++)
		ecp->ecp_command.arm.pf_def.arm_coordinates[j] = final_position[j];

	// printf("\n ilosc krokow: %d, po ilu komun: %d, odleglosc 1: %f\n",ecp_command.motion_steps, ecp_command.value_in_step_no, ecp_command.arm.pf_def.arm_coordinates[1]);

	execute_motion();

	if (ecp->is_synchronised())
		for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
			current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
void EcpRobot::move_joints(const double final_position[])
{
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
	ecp->ecp_command.instruction_type = lib::SET_GET;
	ecp->ecp_command.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.get_arm_type = lib::JOINT;
	ecp->ecp_command.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.set_arm_type = lib::JOINT;
	ecp->ecp_command.motion_type = lib::ABSOLUTE;
	ecp->ecp_command.interpolation_type = lib::MIM;
	ecp->ecp_command.motion_steps = nr_of_steps;
	ecp->ecp_command.value_in_step_no = nr_of_steps;

	// cprintf("NOS=%u\n",ecp_command.motion_steps);

	if (nr_of_steps < 1) // Nie wykowywac bo zadano ruch do aktualnej pozycji
		return;

	for (int j = 0; j < ecp->number_of_servos; j++)
		ecp->ecp_command.arm.pf_def.arm_coordinates[j] = final_position[j];

	execute_motion();

	for (int j = 0; j < ecp->number_of_servos; j++) // Przepisanie aktualnych polozen
		current_position[j] = ecp->reply_package.arm.pf_def.arm_coordinates[j];
}
// ---------------------------------------------------------------

}
} //namespace ui
} //namespace mrrocpp



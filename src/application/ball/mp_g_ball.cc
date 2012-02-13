// -------------------------------------------------------------------------
//
// MP Master Process - methods for force generators
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <cstdio>
#include <iostream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/mp/mp_robot.h"
#include "application/ball/mp_g_ball.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/mp/generator/mp_g_wait_for_task_termination.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace generator {

ball::ball(task::task& _mp_task, int step) :
		continously_coordinated(_mp_task),
		irp6ot_con(true),
		irp6p_con(true),
		global_base(1, 0, 0, -0.08, 0, 1, 0, 2.08, 0, 0, 1, -0.015),
		speedup(0.0),
		speedup_factor(0.005)
{
	step_no = step;
}

void ball::configure(bool l_irp6ot_con, bool l_irp6p_con)
{
	irp6ot_con = l_irp6ot_con;
	irp6p_con = l_irp6p_con;
}

void ball::setup_command(robot::robot & robot)
{
	lib::trajectory_description td;

	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 4;

	robot.mp_command.command = lib::NEXT_POSE;
	robot.mp_command.instruction.instruction_type = lib::GET;
	robot.mp_command.instruction.get_type = ARM_DEFINITION;
	robot.mp_command.instruction.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	robot.mp_command.instruction.robot_model.type = lib::TOOL_FRAME;
	robot.mp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	robot.mp_command.instruction.set_arm_type = lib::FRAME;
	robot.mp_command.instruction.motion_type = lib::ABSOLUTE;
	robot.mp_command.instruction.interpolation_type = lib::TCIM;
	robot.mp_command.instruction.motion_steps = td.internode_step_no;
	robot.mp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i = 0; i < 3; i++) {
		robot.mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		robot.mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		robot.mp_command.instruction.arm.pf_def.arm_coordinates[i + 3] = 0;
		robot.mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i + 3] = 0;
		robot.mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
		robot.mp_command.instruction.arm.pf_def.reciprocal_damping[i + 3] = lib::TORQUE_RECIPROCAL_DAMPING;
		robot.mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		robot.mp_command.instruction.arm.pf_def.behaviour[i + 3] = lib::UNGUARDED_MOTION;
		robot.mp_command.instruction.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		robot.mp_command.instruction.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;
	}

	// define virtual tool position
	const lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	robot.mp_command.instruction.robot_model.tool_frame_def.tool_frame = tool_frame;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step ---------------------------------
// ----------------------------------------------------------------------------------------------

bool ball::first_step()
{
	std::cout << "first_step" << std::endl;

	irp6ot = robot_m[lib::irp6ot_m::ROBOT_NAME];
	irp6p = robot_m[lib::irp6p_m::ROBOT_NAME];

	irp6ot->communicate_with_ecp = true;
	irp6p->communicate_with_ecp = true;

	setup_command(*irp6ot);
	setup_command(*irp6p);

	irp6ot->mp_command.instruction.arm.pf_def.behaviour[2] = lib::CONTACT;
	irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[2] = 12.5;

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ball::next_step_inside()
{
	// Oczekiwanie na odczyt aktualnego polozenia koncowki
	if (cycle_counter < 3) {
		return true;
	}

	if (check_and_null_trigger()) {
		return false;
	}

	// przestawienie się na zapis i odczyt
	if (cycle_counter == 3) {
		irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;

		irp6ot_start = irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame;
		irp6p_start = irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame;
	}

	// trajectory generation helper variables
	lib::Xyz_Angle_Axis_vector aa_vector;

	const double t = speedup * 2 * M_PI * cycle_counter / 800;

	speedup += speedup_factor;
	if (speedup > 1.0) {
		speedup = 1.0;
	}

	// IRP6 on track

	// homogeneous transformation matrix -> angle axis vector
	irp6ot_start.get_xyz_angle_axis(aa_vector);

	// actual command
	aa_vector[0] = 0.1 * sin(t);
	aa_vector[1] = 0.920;
	aa_vector[2] = 0.165 + 0.1 * cos(t);

	//	std::cout << aa_vector << std::endl;

	// angle axis vector -> homogeneous transformation matrix
	irp6ot->mp_command.instruction.arm.pf_def.arm_frame.set_from_xyz_angle_axis(aa_vector);

	// IRP6 postument

	// homogeneous transformation matrix -> angle axis vector
	irp6p_start.get_xyz_angle_axis(aa_vector);

	// actual command
	aa_vector[0] = -0.106 + 0.1 * sin(t);
	aa_vector[1] = 1.087 + 0.1 * sin(t);
	aa_vector[2] = 0.135 + 0.1 * cos(t);

	//	std::cout << aa_vector << std::endl;

	// angle axis vector -> homogeneous transformation matrix
	irp6p->mp_command.instruction.arm.pf_def.arm_frame.set_from_xyz_angle_axis(aa_vector);

	return true;

	if ((irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED)
			|| (irp6p->ecp_reply_package.reply == lib::TASK_TERMINATED)) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else
		return true;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


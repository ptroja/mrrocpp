// -------------------------------------------------------------------------
//
// MP Master Process - methods for force generators
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <stdio.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "application/ball/mp_g_ball.h"
#include "lib/mrmath/mrmath.h"
#include "mp/generator/mp_g_common.h"

namespace mrrocpp {
namespace mp {
namespace generator {


ball::ball(task::task& _mp_task, int step) :
	generator(_mp_task),
	irp6ot_con(true), irp6p_con(true),
	global_base(1, 0, 0, -0.08, 0, 1, 0, 2.08, 0, 0, 1, -0.015),
	speedup(0.0), speedup_factor(0.005)
{
	step_no = step;
}

void ball::configure(bool l_irp6ot_con, bool l_irp6p_con)
{
	irp6ot_con = l_irp6ot_con;
	irp6p_con = l_irp6p_con;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step ---------------------------------
// ----------------------------------------------------------------------------------------------

bool ball::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	std::cout << "first_step" << std::endl;
	irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	irp6p = robot_m[lib::ROBOT_IRP6_POSTUMENT];

	irp6ot->communicate = true;
	irp6p->communicate = true;

	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 4;
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DEFINITION;
	irp6ot->mp_command.instruction.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	irp6ot->mp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.set_arm_type = lib::FRAME;
	irp6ot->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6ot->mp_command.instruction.motion_type = lib::ABSOLUTE;
	irp6ot->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<3; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::UNGUARDED_MOTION;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}
	irp6ot->mp_command.instruction.arm.pf_def.behaviour[2] = lib::CONTACT;
	irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[2] = 12.5;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(irp6ot->mp_command.instruction.robot_model.tool_frame_def.tool_frame);

	irp6p->mp_command.command = lib::NEXT_POSE;
	irp6p->mp_command.instruction.instruction_type = lib::GET;
	irp6p->mp_command.instruction.get_type = ARM_DEFINITION;
	irp6p->mp_command.instruction.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	irp6p->mp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.set_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.motion_type = lib::ABSOLUTE;
	irp6p->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(irp6p->mp_command.instruction.robot_model.tool_frame_def.tool_frame);

	for (int i=0; i<3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::GUARDED_MOTION;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i+3] = lib::GUARDED_MOTION;

		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool ball::next_step()
{
//	std::cout << "next_step" << std::endl;

	if (node_counter<3) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		return true;
	}

	if (check_and_null_trigger()) {
		return false;
	}

	// przestawienie się na zapis i odczyt
	if (node_counter==3) {
		irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;

		irp6ot->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6ot->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
		irp6p->mp_command.instruction.arm.pf_def.gripper_coordinate = irp6p->ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;

		irp6ot_start.set_from_frame_tab(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
		irp6p_start.set_from_frame_tab(irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
	}

	// trajectory generation helper variables
	lib::Homog_matrix hm;
	lib::Xyz_Angle_Axis_vector aa_vector;
	const double t = speedup*2*M_PI*node_counter/800;

	speedup += speedup_factor;
	if(speedup > 1.0) {
		speedup = 1.0;
	}

	// IRP6 on track

	// frame_tab -> homogeneous transformation matrix
	hm = irp6ot_start;

	// homogeneous transformation matrix -> angle axis vector
	hm.get_xyz_angle_axis(aa_vector);

	// actual command
	aa_vector[0] = 0.1*sin(t);
	aa_vector[1] = 0.920;
	aa_vector[2] = 0.165+0.1*cos(t);

//	std::cout << aa_vector << std::endl;

	// angle axis vector -> homogeneous transformation matrix
	hm.set_from_xyz_angle_axis(aa_vector);

	// homogeneous transformation matrix -> frame_tab
	hm.get_frame_tab(irp6ot->mp_command.instruction.arm.pf_def.arm_frame);


	// IRP6 postument

	// frame_tab -> homogeneous transformation matrix
	hm = irp6p_start;

	// homogeneous transformation matrix -> angle axis vector
	hm.get_xyz_angle_axis(aa_vector);

	// actual command
	aa_vector[0] = -0.106+0.1*sin(t);
	aa_vector[1] = 1.087+0.1*sin(t);
	aa_vector[2] = 0.135+0.1*cos(t);

//	std::cout << aa_vector << std::endl;

	// angle axis vector -> homogeneous transformation matrix
	hm.set_from_xyz_angle_axis(aa_vector);

	// homogeneous transformation matrix -> frame_tab
	hm.get_frame_tab(irp6p->mp_command.instruction.arm.pf_def.arm_frame);

	return true;
/*
	lib::Homog_matrix irp6ot_current_arm_frame(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
	lib::Homog_matrix irp6p_current_arm_frame(irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame);

	lib::Homog_matrix irp6p_goal_frame(global_base*irp6ot_current_arm_frame);
	irp6p_goal_frame.get_frame_tab(irp6p->mp_command.instruction.arm.pf_def.arm_frame);
*/
	/*
	 lib::Homog_matrix irp6p_goal_frame_increment_in_end_effector ((!irp6p_current_arm_frame)*irp6p_goal_frame);
	 lib::Ft_v_vector irp6p_goal_xyz_angle_axis_increment_in_end_effector;

	 irp6p_goal_frame_increment_in_end_effector.get_xyz_angle_axis(irp6p_goal_xyz_angle_axis_increment_in_end_effector);

	 irp6p_goal_xyz_angle_axis_increment_in_end_effector=irp6p_goal_xyz_angle_axis_increment_in_end_effector *
	 (double) (1/ ( ((double)STEP)*((double)step_no)*2) );

	 irp6p_goal_xyz_angle_axis_increment_in_end_effector.to_table (irp6p->ecp_td.MPtoECP_position_velocity);
	 */
	//	irp6p->ecp_td.MPtoECP_position_velocity[2] = 0.01;

	/*
	lib::Ft_v_vector irp6p_ECPtoMP_force_xyz_torque_xyz(irp6p->ecp_reply_package.reply_package.arm.pf_def.force_xyz_torque_xyz);

	for (int i=0; i<6; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = -irp6p_ECPtoMP_force_xyz_torque_xyz[i];
	}

	if ( (node_counter % 10) == 0) {
		std::cout << "irp6p_ECPtoMP_force_xyz_torque_xyz\n" << irp6p_ECPtoMP_force_xyz_torque_xyz << std::endl;
		//	std::cout << "irp6p_goal_xyz_angle_axis_increment_in_end_effector\n" << irp6p_goal_xyz_angle_axis_increment_in_end_effector << std::endl;
	}
	*/

	if ((irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED ) || (irp6p->ecp_reply_package.reply == lib::TASK_TERMINATED )) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	} else
		return true;
}


} // namespace generator
} // namespace mp
} // namespace mrrocpp


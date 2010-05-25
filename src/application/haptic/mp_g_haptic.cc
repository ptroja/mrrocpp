// -------------------------------------------------------------------------
//
// MP Master Process - methods���for force generators
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
#include "application/haptic/mp_g_haptic.h"
#include "lib/mrmath/mrmath.h"
#include "mp/generator/mp_g_common.h"

namespace mrrocpp {
namespace mp {
namespace generator {

haptic::haptic(task::task& _mp_task, int step) :
	generator(_mp_task), irp6ot_con(1), irp6p_con(1), global_base(1, 0, 0,
			-0.08, 0, 1, 0, 2.08, 0, 0, 1, -0.015) {
	step_no = step;
}

void haptic::configure(unsigned short l_irp6ot_con, unsigned short l_irp6p_con) {
	irp6ot_con = l_irp6ot_con;
	irp6p_con = l_irp6p_con;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step ---------------------------------
// ----------------------------------------------------------------------------------------------

bool haptic::first_step() {
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;
	irp6ot = robot_m[lib::ROBOT_IRP6OT_M];
	irp6p = robot_m[lib::ROBOT_IRP6P_M];

	irp6ot->communicate = true;
	irp6p->communicate = true;

	irp6ot->continuous_coordination = true;
	irp6p->continuous_coordination = true;

	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DEFINITION;
	irp6ot->mp_command.instruction.set_type = ARM_DEFINITION
			| ROBOT_MODEL_DEFINITION;
	irp6ot->mp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6ot->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6ot->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6ot->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i = 0; i < 3; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i + 3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i + 3]
				= 0;
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i]
				= FORCE_RECIPROCAL_DAMPING / 2;
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i + 3]
				= TORQUE_RECIPROCAL_DAMPING / 2;
		//		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING / 40;
		//		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING / 40;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i + 3]
				= lib::CONTACT;
		/*
		 if(irp6ot_con) irp6ot->ecp_td.MPtoECP_reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
		 else irp6ot->ecp_td.MPtoECP_reciprocal_damping[i] = 0.0;
		 if(irp6ot_con) irp6ot->ecp_td.MPtoECP_reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
		 else irp6ot->ecp_td.MPtoECP_reciprocal_damping[i+3] = 0.0;
		 */
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i + 3]
				= TORQUE_INERTIA;

		//		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = 0;
		//		irp6ot->mp_command.instruction.arm.pf_def.inertia[i+3] = 0;

	}

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(
			irp6ot->mp_command.instruction.robot_model.tool_frame_def.tool_frame);

	irp6p->mp_command.command = lib::NEXT_POSE;
	irp6p->mp_command.instruction.instruction_type = lib::GET;
	irp6p->mp_command.instruction.get_type = ARM_DEFINITION;
	irp6p->mp_command.instruction.set_type = ARM_DEFINITION
			| ROBOT_MODEL_DEFINITION;
	irp6p->mp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.set_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.get_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.motion_type = lib::ABSOLUTE;
	irp6p->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(
			irp6p->mp_command.instruction.robot_model.tool_frame_def.tool_frame);

	for (int i = 0; i < 3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i + 3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i + 3]
				= 0;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i]
				= FORCE_RECIPROCAL_DAMPING / 2;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i + 3]
				= TORQUE_RECIPROCAL_DAMPING / 2;
		//		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING/40;
		//		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = FORCE_RECIPROCAL_DAMPING/40;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i]
				= lib::GUARDED_MOTION;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i + 3]
				= lib::GUARDED_MOTION;
		/*
		 if(irp6p_con) irp6p->ecp_td.MPtoECP_reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
		 else irp6p->ecp_td.MPtoECP_reciprocal_damping[i] = 0.0;
		 if(irp6p_con) irp6p->ecp_td.MPtoECP_reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
		 else irp6p->ecp_td.MPtoECP_reciprocal_damping[i+3] = 0.0;
		 */

		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i + 3]
				= TORQUE_INERTIA;

		//		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = 0;
		//		irp6p->mp_command.instruction.arm.pf_def.inertia[i+3] = 0;

		/*
		 irp6p->ecp_td.MPtoECP_inertia[i] = 0.0;
		 irp6p->ecp_td.MPtoECP_inertia[i+3] = 0.0;
		 */
	}

	//	  cout << "first_step 3" << endl;
	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool haptic::next_step() {
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	//	cout << "next_step" << endl;

	if (node_counter < 3) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		return true;
	}

	if (check_and_null_trigger()) {
		return false;
	}

	if (node_counter == 3) {

		irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;

	}

	lib::Homog_matrix irp6ot_current_arm_frame(
			irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
	lib::Homog_matrix irp6p_current_arm_frame(
			irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame);

	lib::Homog_matrix irp6p_goal_frame(global_base * irp6ot_current_arm_frame);
	irp6p_goal_frame.get_frame_tab(
			irp6p->mp_command.instruction.arm.pf_def.arm_frame);

	/*
	 lib::Homog_matrix irp6p_goal_frame_increment_in_end_effector ((!irp6p_current_arm_frame)*irp6p_goal_frame);
	 lib::Ft_v_vector irp6p_goal_xyz_angle_axis_increment_in_end_effector;

	 irp6p_goal_frame_increment_in_end_effector.get_xyz_angle_axis(irp6p_goal_xyz_angle_axis_increment_in_end_effector);

	 irp6p_goal_xyz_angle_axis_increment_in_end_effector=irp6p_goal_xyz_angle_axis_increment_in_end_effector *
	 (double) (1/ ( ((double)STEP)*((double)step_no)*2) );

	 irp6p_goal_xyz_angle_axis_increment_in_end_effector.to_table (irp6p->ecp_td.MPtoECP_position_velocity);
	 */
	//	irp6p->ecp_td.MPtoECP_position_velocity[2] = 0.01;

	lib::Ft_v_vector
			irp6p_ECPtoMP_force_xyz_torque_xyz(
					irp6p->ecp_reply_package.reply_package.arm.pf_def.force_xyz_torque_xyz);

	for (int i = 0; i < 6; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i]
				= -irp6p_ECPtoMP_force_xyz_torque_xyz[i];
	}

	// modyfikacja dlugosci makrokroku postumenta na podstawie analizy wyprzedzenia pulse z ECP postumenta wzgledem pulsu z ECP traka
	// sam proces korekty jest konieczny ze wzgledu na to ze przerwanie w EDP traka dochodzi co okolo 2,08 ms zamiast 2ms w postumecie i calosc sie rozjezdza.

	float time_interval = (irp6ot->pulse_receive_time.tv_sec
			+ irp6ot->pulse_receive_time.tv_nsec / 1e9)
			- (irp6p->pulse_receive_time.tv_sec
					+ irp6p->pulse_receive_time.tv_nsec / 1e9);

	if (time_interval > 0.002) {
		irp6p->mp_command.instruction.motion_steps = step_no + 1;
		irp6p->mp_command.instruction.value_in_step_no = step_no + 1 - 4;
	} else {
		irp6p->mp_command.instruction.motion_steps = step_no;
		irp6p->mp_command.instruction.value_in_step_no = step_no - 4;
	}

	if ((node_counter % 10) == 0) {
		std::cout << "irp6p_ECPtoMP_force_xyz_torque_xyz\n"
				<< irp6p_ECPtoMP_force_xyz_torque_xyz << "interval:"
				<< time_interval << std::endl;
		//	std::cout << "irp6p_goal_xyz_angle_axis_increment_in_end_effector\n" << irp6p_goal_xyz_angle_axis_increment_in_end_effector << std::endl;

	}

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


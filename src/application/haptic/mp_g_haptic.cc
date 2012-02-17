// -------------------------------------------------------------------------
//
// MP Master Process - methods���for force generators
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <cstdio>
#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp> //include all types plus i/o
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/mp/mp_robot.h"
#include "application/haptic/mp_g_haptic.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/mp/generator/mp_g_wait_for_task_termination.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace generator {

haptic::haptic(task::task& _mp_task, int step) :
		continously_coordinated(_mp_task), global_base(1, 0, 0, -0.08, 0, 1, 0, 2.08, 0, 0, 1, -0.015)
{
	step_no = step;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step ---------------------------------
// ----------------------------------------------------------------------------------------------

bool haptic::first_step()
{
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana

	irp6ot = robot_m[lib::irp6ot_m::ROBOT_NAME];
	irp6p = robot_m[lib::irp6p_m::ROBOT_NAME];

	irp6ot->communicate_with_ecp = true;
	irp6p->communicate_with_ecp = true;

	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 1;
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DEFINITION;
	irp6ot->mp_command.instruction.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	irp6ot->mp_command.instruction.robot_model.type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	irp6ot->mp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	irp6ot->mp_command.instruction.motion_type = lib::RELATIVE;
	irp6ot->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6ot->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6ot->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz = lib::Ft_vector::Zero();

	for (int i = 0; i < 3; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i + 3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING / 2;
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i + 3] = lib::TORQUE_RECIPROCAL_DAMPING / 2;
		//		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING / 40;
		//		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = lib::TORQUE_RECIPROCAL_DAMPING / 40;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i + 3] = lib::CONTACT;
		/*
		 if(irp6ot_con) irp6ot->ecp_td.MPtoECP_reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
		 else irp6ot->ecp_td.MPtoECP_reciprocal_damping[i] = 0.0;
		 if(irp6ot_con) irp6ot->ecp_td.MPtoECP_reciprocal_damping[i+3] = lib::TORQUE_RECIPROCAL_DAMPING;
		 else irp6ot->ecp_td.MPtoECP_reciprocal_damping[i+3] = 0.0;
		 */
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;

		//		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = 0;
		//		irp6ot->mp_command.instruction.arm.pf_def.inertia[i+3] = 0;

	}

	const lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	irp6ot->mp_command.instruction.robot_model.tool_frame_def.tool_frame = tool_frame;

	irp6p->mp_command.command = lib::NEXT_POSE;
	irp6p->mp_command.instruction.instruction_type = lib::GET;
	irp6p->mp_command.instruction.get_type = ARM_DEFINITION;
	irp6p->mp_command.instruction.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	irp6p->mp_command.instruction.robot_model.type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	irp6p->mp_command.instruction.set_arm_type = lib::FRAME;
	irp6p->mp_command.instruction.motion_type = lib::ABSOLUTE;
	irp6p->mp_command.instruction.interpolation_type = lib::TCIM;
	irp6p->mp_command.instruction.motion_steps = td.internode_step_no;
	irp6p->mp_command.instruction.value_in_step_no = td.value_in_step_no;

	irp6p->mp_command.instruction.robot_model.tool_frame_def.tool_frame = tool_frame;

	irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz = lib::Ft_vector::Zero();

	for (int i = 0; i < 3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;

		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i + 3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING / 2;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i + 3] = lib::TORQUE_RECIPROCAL_DAMPING / 2;
		//		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING/40;
		//		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = lib::FORCE_RECIPROCAL_DAMPING/40;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::GUARDED_MOTION;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i + 3] = lib::GUARDED_MOTION;
		/*
		 if(irp6p_con) irp6p->ecp_td.MPtoECP_reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
		 else irp6p->ecp_td.MPtoECP_reciprocal_damping[i] = 0.0;
		 if(irp6p_con) irp6p->ecp_td.MPtoECP_reciprocal_damping[i+3] = lib::TORQUE_RECIPROCAL_DAMPING;
		 else irp6p->ecp_td.MPtoECP_reciprocal_damping[i+3] = 0.0;
		 */

		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;

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

bool haptic::next_step_inside()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	//	cout << "next_step" << endl;

	if (cycle_counter < 3) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		return true;
	}

	if (check_and_null_trigger()) {
		return false;
	}

	if (cycle_counter == 3) {

		irp6ot->mp_command.instruction.instruction_type = lib::SET_GET;
		irp6p->mp_command.instruction.instruction_type = lib::SET_GET;

	}

	const lib::Homog_matrix & irp6ot_current_arm_frame = irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame;

	// not used
	//const lib::Homog_matrix & irp6p_current_arm_frame = irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame;

	const lib::Homog_matrix irp6p_goal_frame = global_base * irp6ot_current_arm_frame;
	irp6p->mp_command.instruction.arm.pf_def.arm_frame = irp6p_goal_frame;

	/*
	 lib::Homog_matrix irp6p_goal_frame_increment_in_end_effector ((!irp6p_current_arm_frame)*irp6p_goal_frame);
	 lib::Ft_v_vector irp6p_goal_xyz_angle_axis_increment_in_end_effector;

	 irp6p_goal_frame_increment_in_end_effector.get_xyz_angle_axis(irp6p_goal_xyz_angle_axis_increment_in_end_effector);

	 irp6p_goal_xyz_angle_axis_increment_in_end_effector=irp6p_goal_xyz_angle_axis_increment_in_end_effector *
	 (double) (1/ ( ((double)STEP)*((double)step_no)*2) );

	 irp6p_goal_xyz_angle_axis_increment_in_end_effector.to_table (irp6p->ecp_td.MPtoECP_position_velocity);
	 */
	//	irp6p->ecp_td.MPtoECP_position_velocity[2] = 0.01;
	const lib::Ft_vector & irp6p_ECPtoMP_force_xyz_torque_xyz =
			irp6p->ecp_reply_package.reply_package.arm.pf_def.force_xyz_torque_xyz;

	irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz = -irp6p_ECPtoMP_force_xyz_torque_xyz;

	// modyfikacja dlugosci makrokroku postumenta na podstawie analizy wyprzedzenia pulse z ECP postumenta wzgledem pulsu z ECP traka
	// sam proces korekty jest konieczny ze wzgledu na to ze przerwanie w EDP traka dochodzi co okolo 2,08 ms zamiast 2ms w postumecie i calosc sie rozjezdza.

	boost::posix_time::time_duration time_interval = irp6ot->reply.getTimestamp() - irp6p->reply.getTimestamp();

	/*
	 if (time_interval > boost::posix_time::millisec(2)) {
	 irp6p->mp_command.instruction.motion_steps = step_no + 1;
	 irp6p->mp_command.instruction.value_in_step_no = step_no - 4 + 1;
	 } else {
	 irp6p->mp_command.instruction.motion_steps = step_no;
	 irp6p->mp_command.instruction.value_in_step_no = step_no - 4;
	 }
	 */

	//std::cout << time_interval << std::endl;
	/*
	 if ((cycle_counter % 10) == 0) {
	 std::cout << "irp6p_ECPtoMP_force_xyz_torque_xyz\n" << irp6p_ECPtoMP_force_xyz_torque_xyz << "interval:"
	 << time_interval << std::endl << irp6p_goal_frame << std::endl;
	 //	std::cout << "irp6p_goal_xyz_angle_axis_increment_in_end_effector\n" << irp6p_goal_xyz_angle_axis_increment_in_end_effector << std::endl;

	 }
	 */

	if ((irp6ot->ecp_reply_package.reply == lib::TASK_TERMINATED)
			|| (irp6p->ecp_reply_package.reply == lib::TASK_TERMINATED)) {
		sr_ecp_msg.message("w mp task terminated");
		return false;
	}

	return true;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


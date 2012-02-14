// -------------------------------------------------------------------------
//
// MP Master Process - methods for force generators
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <cstdio>
#include <iostream>

#include <boost/date_time/posix_time/posix_time_types.hpp> //no i/o just types
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/mp/mp_robot.h"
#include "application/haptic_stiffness/mp_g_haptic_stiffness.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/mp/generator/mp_g_wait_for_task_termination.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace generator {

haptic_stiffness::haptic_stiffness(task::task& _mp_task, int step) :
		continously_coordinated(_mp_task),
		irp6p_state(HS_LOW_FORCE),
		total_irp6p_stiffness(0.0),
		last_irp6p_stiffness(0.0),
		initial_irp6p_force(0.0),
		initial_irp6p_position(0.0),
		intermediate_irp6p_force(0.0),
		intermediate_irp6p_position(0.0),
		irp6ot_con(1),
		irp6p_con(1),
		global_base(1, 0, 0, -0.08, 0, 1, 0, 2.08, 0, 0, 1, -0.015)
{
	step_no = step;
}

void haptic_stiffness::configure(unsigned short l_irp6ot_con, unsigned short l_irp6p_con)
{
	irp6ot_con = l_irp6ot_con;
	irp6p_con = l_irp6p_con;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step ---------------------------------
// ----------------------------------------------------------------------------------------------

bool haptic_stiffness::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;
	irp6ot = robot_m[lib::irp6ot_m::ROBOT_NAME];
	irp6p = robot_m[lib::irp6p_m::ROBOT_NAME];

	irp6ot->communicate_with_ecp = true;
	irp6p->communicate_with_ecp = true;

	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;
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

	for (int i = 0; i < 3; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates[i + 3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i + 3] = 0;
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i + 3] = lib::TORQUE_RECIPROCAL_DAMPING / 2;
		//		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING / 40;
		//		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = lib::TORQUE_RECIPROCAL_DAMPING / 40;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		irp6ot->mp_command.instruction.arm.pf_def.behaviour[i + 3] = lib::UNGUARDED_MOTION;
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

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
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

	for (int i = 0; i < 3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		irp6p->mp_command.instruction.arm.pf_def.arm_coordinates[i + 3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i + 3] = 0;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING / 2;
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i + 3] = lib::TORQUE_RECIPROCAL_DAMPING / 2;
		//		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING/40;
		//		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = lib::FORCE_RECIPROCAL_DAMPING/40;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::GUARDED_MOTION;
		irp6p->mp_command.instruction.arm.pf_def.behaviour[i + 3] = lib::UNGUARDED_MOTION;
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

bool haptic_stiffness::next_step_inside()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	//	cout << "next_step" << endl;

	double current_irp6p_force;
	double current_irp6p_position;

	double current_irp6ot_force;
	double current_irp6ot_position;

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

	lib::Homog_matrix irp6ot_current_arm_frame(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame);
	lib::Homog_matrix irp6p_current_arm_frame(irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame);

	lib::Homog_matrix irp6p_goal_frame(global_base * irp6ot_current_arm_frame);
	irp6p->mp_command.instruction.arm.pf_def.arm_frame = irp6p_goal_frame;

	lib::Homog_matrix irp6ot_goal_frame;

	/*
	 lib::Homog_matrix irp6p_goal_frame_increment_in_end_effector ((!irp6p_current_arm_frame)*irp6p_goal_frame);
	 lib::Ft_v_vector irp6p_goal_xyz_angle_axis_increment_in_end_effector;

	 irp6p_goal_frame_increment_in_end_effector.get_xyz_angle_axis(irp6p_goal_xyz_angle_axis_increment_in_end_effector);

	 irp6p_goal_xyz_angle_axis_increment_in_end_effector=irp6p_goal_xyz_angle_axis_increment_in_end_effector *
	 (double) (1/ ( ((double)STEP)*((double)step_no)*2) );

	 irp6p_goal_xyz_angle_axis_increment_in_end_effector.to_table (irp6p->ecp_td.MPtoECP_position_velocity);
	 */
	//	irp6p->ecp_td.MPtoECP_position_velocity[2] = 0.01;
	lib::Ft_v_vector irp6p_ECPtoMP_force_xyz_torque_xyz(irp6p->ecp_reply_package.reply_package.arm.pf_def.force_xyz_torque_xyz);

	for (int i = 0; i < 6; i++) {
		irp6ot->mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = -irp6p_ECPtoMP_force_xyz_torque_xyz[i];
	}

	// modyfikacja dlugosci makrokroku postumenta na podstawie analizy wyprzedzenia pulse z ECP postumenta wzgledem pulsu z ECP traka
	// sam proces korekty jest konieczny ze wzgledu na to ze przerwanie w EDP traka dochodzi co okolo 2,08 ms zamiast 2ms w postumecie i calosc sie rozjezdza.

	boost::posix_time::time_duration time_interval = irp6ot->reply.getTimestamp() - irp6p->reply.getTimestamp();

	if (time_interval > boost::posix_time::millisec(2)) {
		irp6p->mp_command.instruction.motion_steps = step_no + 1;
		irp6p->mp_command.instruction.value_in_step_no = step_no + 1 - 4;
	} else {
		irp6p->mp_command.instruction.motion_steps = step_no;
		irp6p->mp_command.instruction.value_in_step_no = step_no - 4;
	}

	// Estymacja sztywności w automacie skończonym

	// pobranie biezacej sily i polozenia w osi z narzedzia

	current_irp6p_force = irp6p->ecp_reply_package.reply_package.arm.pf_def.force_xyz_torque_xyz[2];

	current_irp6ot_force = irp6ot->ecp_reply_package.reply_package.arm.pf_def.force_xyz_torque_xyz[2];

	lib::Homog_matrix current_irp6p_frame(irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_frame);

	lib::Homog_matrix current_irp6ot_frame(irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_frame);

	current_irp6p_position = current_irp6p_frame(2, 3);
	current_irp6ot_position = current_irp6ot_frame(2, 3);

	switch (irp6p_state)
	{
		case HS_LOW_FORCE:
			//
			for (int i = 0; i < 3; i++) {
				irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
				irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
			}

			//std::cout << "HS_LOW_FORCE" << std::endl;

			if (current_irp6p_force > MINIMAL_FORCE) {
				irp6p_state = HS_STIFNESS_ESTIMATION;
				initial_irp6p_force = current_irp6p_force;
				initial_irp6p_position = current_irp6p_position;

				intermediate_irp6p_force = current_irp6p_force;
				intermediate_irp6p_position = current_irp6p_position;

				//	std::cout << "HS_STIFNESS_ESTIMATION" << std::endl;

				for (int i = 0; i < 3; i++) {
					irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
					irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
					lib::Xyz_Angle_Axis_vector irp6ot_desired_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
					irp6ot_desired_velocity.to_table(irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates);
				}

			}

			break;
		case HS_STIFNESS_ESTIMATION:
			if (current_irp6p_force <= MINIMAL_FORCE) {
				irp6p_state = HS_LOW_FORCE;
				total_irp6p_stiffness = 0.0;
				last_irp6p_stiffness = 0.0;

			} else if ((fabs(current_irp6p_force - initial_irp6p_force) >= FORCE_INCREMENT)
					|| (fabs(current_irp6p_position - initial_irp6p_position) >= POSITION_INCREMENT)) {
				double computed_irp6p_stiffness = (current_irp6p_force - initial_irp6p_force)
						/ -(current_irp6p_position - initial_irp6p_position);
				//	std::cout << "STIFNESS_ESTIMATION" << std::endl;

				if ((fabs(current_irp6p_force - initial_irp6p_force) >= HIGH_FORCE_INCREMENT)
						|| (fabs(current_irp6p_position - initial_irp6p_position) >= HIGH_POSITION_INCREMENT)) {

					for (int i = 0; i < 3; i++) {
						irp6ot->mp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
						irp6p->mp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
						lib::Xyz_Angle_Axis_vector irp6ot_desired_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
						irp6ot_desired_velocity.to_table(irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates);
					}
				}

				if (computed_irp6p_stiffness > 0.0) {
					total_irp6p_stiffness = computed_irp6p_stiffness;
				}

				if ((fabs(current_irp6p_force - intermediate_irp6p_force) >= FORCE_INCREMENT)
						|| (fabs(current_irp6p_position - intermediate_irp6p_position) >= POSITION_INCREMENT)) {

					double computed_intermiediate_irp6p_stiffness = (current_irp6p_force - intermediate_irp6p_force)
							/ -(current_irp6p_position - intermediate_irp6p_position);

					if (computed_intermiediate_irp6p_stiffness > 0.0) {
						last_irp6p_stiffness = computed_intermiediate_irp6p_stiffness;
					}

					intermediate_irp6p_force = current_irp6p_force;
					intermediate_irp6p_position = current_irp6p_position;

				}

			} else {

				lib::Xyz_Angle_Axis_vector irp6ot_desired_velocity(0.0, 0.0, APPROACH_VELOCITY, 0.0, 0.0, 0.0);
				irp6ot_desired_velocity.to_table(irp6ot->mp_command.instruction.arm.pf_def.arm_coordinates);

			}
			break;

	}

	// Korekta parametrów regulatora siłowego w robocie podrzednym na podstawie estymaty sztywnosci
	double divisor;

	if (total_irp6p_stiffness > ADAPTATION_FACTOR) {
		divisor = total_irp6p_stiffness / ADAPTATION_FACTOR;
	} else {
		divisor = 1;
	}

	for (int i = 0; i < 3; i++) {
		irp6p->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING / divisor;
		irp6p->mp_command.instruction.arm.pf_def.inertia[i] = lib::FORCE_INERTIA / divisor;

		irp6ot->mp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING / divisor;
		irp6ot->mp_command.instruction.arm.pf_def.inertia[i] = lib::FORCE_INERTIA / divisor;
	}
	// wypiski

	//	if ((cycle_counter % 10) == 0) {
	std::cout << "irp6p_f: " << current_irp6p_force << ", irp6p_p: " << current_irp6p_position << ", irp6p_ts: "
			<< total_irp6p_stiffness << ", irp6p_ls: " << last_irp6p_stiffness << ", irp6ot_f: " << current_irp6ot_force
			<< ", irp6ot_p: " << current_irp6ot_position << std::endl;

	//std::cout << "irp6p_ECPtoMP_force_xyz_torque_xyz\n" << irp6p_ECPtoMP_force_xyz_torque_xyz << "interval:"
	//		<< time_interval << std::endl;
	//	std::cout << "irp6p_goal_xyz_angle_axis_increment_in_end_effector\n" << irp6p_goal_xyz_angle_axis_increment_in_end_effector << std::endl;

	//	}

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


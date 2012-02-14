/*!
 * @file
 * @brief File contains ecp_generator class definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <cmath>

#include "ecp_g_spring_contact.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			spring_contact_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

spring_contact::spring_contact(common::task::task& _ecp_task, int step) :
		common::generator::generator(_ecp_task), step_no(step), tool_frame(0.0, 0.0, 0.25)
{
	generator_name = ecp_mp::generator::SPRING_CONTACT;
}

bool spring_contact::first_step()
{

	divisor = 1;

	std::cout << std::endl << "spring_contact" << node_counter << std::endl << std::endl;

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	//	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::TOOL_FRAME;
	the_robot->ecp_command.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.set_arm_type = lib::PF_VELOCITY;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = step_no;
	the_robot->ecp_command.value_in_step_no = step_no - 2;

	the_robot->ecp_command.robot_model.tool_frame_def.tool_frame = tool_frame;

	for (int i = 0; i < 3; i++) {
		the_robot->ecp_command.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		the_robot->ecp_command.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;
		the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
		the_robot->ecp_command.arm.pf_def.reciprocal_damping[i + 3] = lib::TORQUE_RECIPROCAL_DAMPING;
	}

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = 0;
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		//	the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0.0;
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	the_robot->ecp_command.arm.pf_def.inertia[2] = lib::FORCE_INERTIA;
	the_robot->ecp_command.arm.pf_def.reciprocal_damping[2] = lib::FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.arm.pf_def.behaviour[2] = lib::CONTACT;
	// Sila dosciku do rawedzi
	the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[2] = 10;

	return true;
}
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
bool spring_contact::next_step()
{

	static bool start_changing_divisor = false;

	double current_irp6p_force;
	double current_irp6p_position;

	// static int count;
	// struct timespec start[9];
	if (check_and_null_trigger()) {
		return false;
	}

	the_robot->ecp_command.instruction_type = lib::SET_GET;

	// pobranie biezacej sily i polozenia w osi z narzedzia

	current_irp6p_force = the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[2];

	lib::Homog_matrix current_irp6p_frame(the_robot->reply_package.arm.pf_def.arm_frame);

	current_irp6p_position = current_irp6p_frame(2, 3);

	switch (irp6p_state)
	{
		case HS_LOW_FORCE:
			//
			//std::cout << "HS_LOW_FORCE" << std::endl;

			if (current_irp6p_force > MINIMAL_FORCE) {
				irp6p_state = HS_STIFNESS_ESTIMATION;
				initial_irp6p_force = current_irp6p_force;
				initial_irp6p_position = current_irp6p_position;

				intermediate_irp6p_force = current_irp6p_force;
				intermediate_irp6p_position = current_irp6p_position;

				//	std::cout << "HS_STIFNESS_ESTIMATION" << std::endl;

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

				}

				if (computed_irp6p_stiffness > 0.0) {
					total_irp6p_stiffness = computed_irp6p_stiffness;
				}

				if ((fabs(current_irp6p_force - intermediate_irp6p_force) >= FORCE_INCREMENT)
						|| (fabs(current_irp6p_position - intermediate_irp6p_position) >= POSITION_INCREMENT)) {

					//if (1) {

					double computed_intermiediate_irp6p_stiffness = (current_irp6p_force - intermediate_irp6p_force)
							/ -(current_irp6p_position - intermediate_irp6p_position);

					if (computed_intermiediate_irp6p_stiffness > 0.0) {
						last_irp6p_stiffness = computed_intermiediate_irp6p_stiffness;
					}

					intermediate_irp6p_force = current_irp6p_force;
					intermediate_irp6p_position = current_irp6p_position;

				}

			} else {

			}
			break;

	}

// #define ADAPTATION 1

#ifdef ADAPTATION
	double adaptation_factor = (HIGHEST_STIFFNESS - STIFFNESS_INSENSIVITY_LEVEL) / (MAX_DIVIDER - 1);

	if (total_irp6p_stiffness > STIFFNESS_INSENSIVITY_LEVEL) {

		if (total_irp6p_stiffness < HIGHEST_STIFFNESS) {

			divisor = 1 + (total_irp6p_stiffness - STIFFNESS_INSENSIVITY_LEVEL) / adaptation_factor;

		} else {
			divisor = MAX_DIVIDER;
		}
	} else {
		divisor = 1;
	}

#else

	if (current_irp6p_force > 10.0) {
		start_changing_divisor = true;

	}

	if (start_changing_divisor) {

		if (node_counter % 1000 == 0) {

			divisor *= 2;

			std::stringstream ss(std::stringstream::in | std::stringstream::out);

			ss << "divisor: " << divisor;

			sr_ecp_msg.message(ss.str().c_str());

		}

	}

#endif
	for (int i = 0; i < 3; i++) {
		the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = 2 * lib::FORCE_RECIPROCAL_DAMPING / (divisor);
		//the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = 2 * lib::FORCE_RECIPROCAL_DAMPING;
		the_robot->ecp_command.arm.pf_def.inertia[i] = 2 * lib::FORCE_INERTIA / divisor;
		//the_robot->ecp_command.arm.pf_def.inertia[i] = 2 * lib::FORCE_INERTIA;
	}
	// wypiski

	//	if ((cycle_counter % 10) == 0) {
	std::cout << "irp6p_f: " << current_irp6p_force << ", irp6p_p: " << current_irp6p_position << ", irp6p_ts: "
			<< total_irp6p_stiffness << ", irp6p_ls: " << last_irp6p_stiffness << ", divisor: " << divisor << std::endl;

	return true;

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

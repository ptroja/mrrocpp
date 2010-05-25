/*
 * $Id$
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_g_mboryn.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 30

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace generator {

const char ecp_g_mboryn::configSectionName[] = { "[mboryn_servovision]" };

ecp_g_mboryn::ecp_g_mboryn(mrrocpp::ecp::common::task::task & _ecp_task)
//: mrrocpp::ecp::common::generator::generator(_ecp_task)
:
	generator(_ecp_task), logEnabled(true)
{
	char kp_name[] = { "kp" };
	char maxt_name[] = { "maxt" };

	if (_ecp_task.config.exists(std::string(kp_name), std::string(configSectionName))) {
		Kp = _ecp_task.config.value <double> (kp_name, configSectionName);
	} else {
		Kp = 0.0001;
		log("Parameter %s->%s not found. Using default value: %g\n", configSectionName, kp_name);
	}

	if (_ecp_task.config.exists(std::string(maxt_name), std::string(configSectionName))) {
		maxT = _ecp_task.config.value <double> (maxt_name, configSectionName);
	} else {
		maxT = 0.01;
		log("Parameter %s->%s not found. Using default value: %g\n", configSectionName, maxt_name, maxT);
	}

	log("\nKp: %g; maxT: %g\n", Kp, maxT);
}

ecp_g_mboryn::~ecp_g_mboryn()
{
}

bool ecp_g_mboryn::first_step()
{
	log("ecp_g_mboryn::first_step()\n");
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 3;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	currentFrameSaved = false;

	log("ecp_g_mboryn::first_step() end\n");

	printInstruction();

	return true;
}

bool ecp_g_mboryn::next_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	if (!currentFrameSaved) { // save first frame
		currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		currentGripperCoordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;

		currentFrameSaved = true;
	}

	// calculate translation
	lib::Xyz_Angle_Axis_vector l_vector;
	currentFrame.get_xyz_angle_axis(l_vector);

	if (vsp_fradia->from_vsp.vsp_report == lib::VSP_REPLY_OK) {
		if (vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.tracking) {

			mrrocpp::lib::K_vector
					e(-vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.y, -vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.x, vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z); // error [pixels]

			for (int i = 0; i < 3; ++i) {
				log("e[%d] = %g\t", i, e[i]);
			}
			log("\n");

			//printInstruction();

			mrrocpp::lib::K_vector u;
			u = e * Kp;
			u(2, 0) = 0; // Z axis

			//for(int i=0; i<3; ++i){		log("u[%d] = %g\t", i, u[i]);	}		log("\n");
			bool isConstrained = false;
			for (int i = 0; i < 3; ++i) {
				if (u(i, 0) > maxT) {
					//log("u[%d] = %g > maxT = %g\n", i, u[i], maxT );
					u(i, 0) = maxT;
					isConstrained = true;
				}
				if (u(i, 0) < -maxT) {
					//log("u[%d] = %g < -maxT = %g\n", i, u[i], -maxT );
					u(i, 0) = -maxT;
					isConstrained = true;
				}

				l_vector(i, 0) += u(i, 0); // first 3 elements of l_vector[] are XYZ translation
			}
			if (isConstrained) {
				log("u CONSTRAINED.\n");
			} else {
				//log("u NOT CONSTRAINED.\n");
			}

			//translation += u;
		} else {
			//log("Not tracking.\n");
		}
	} else {

	}

	lib::Homog_matrix nextFrame;
	nextFrame.set_from_xyz_angle_axis(l_vector);

	// set next frame
	if (isArmFrameOk(nextFrame)) {
		currentFrame = nextFrame;
		nextFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	} else {
		log("!isArmFrameOk(nextFrame)\n");
		currentFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	}
	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = currentGripperCoordinate;

	return true;
} // next_step()

bool ecp_g_mboryn::isArmFrameOk(const lib::Homog_matrix& arm_frame)
{
	lib::K_vector minT(0.600, -0.400, 0.200), maxT(0.950, 0.400, 0.300);
	lib::Xyz_Angle_Axis_vector l_vector;
	arm_frame.get_xyz_angle_axis(l_vector);

	for (int i = 0; i < 3; ++i) {
		if (l_vector(i, 0) < minT(i, 0) || l_vector(i, 0) > maxT(i, 0)) {
			//log("l_vector[%d] = %g not in range: %g ... %g\n", i, l_vector[i], minT[i], maxT[i]);
			return false;
		}
	}

	return true;
}

void ecp_g_mboryn::printInstruction(void)
{
	fflush(stdout);

	cout << "ecp_g_mboryn::printInstruction(void) begin \n";
	cout << "============================================================================================\n";

	cout << "arm.pf_def.arm_frame: " << the_robot->ecp_command.instruction.arm.pf_def.arm_frame << endl;

	for (int i = 0; i < MAX_SERVOS_NR; ++i) {
		cout << "arm.pf_def.arm_coordinates[" << i << "]: "
				<< the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] << endl;
	}

	for (int i = 0; i < MAX_SERVOS_NR; ++i) {
		cout << "arm.pf_def.desired_torque[" << i << "]: "
				<< the_robot->ecp_command.instruction.arm.pf_def.desired_torque[i] << endl;
	}

	for (int i = 0; i < 6; ++i) {
		cout << "arm.pf_def.behaviour[" << i << "]: " << the_robot->ecp_command.instruction.arm.pf_def.behaviour[i]
				<< endl;
	}

	for (int i = 0; i < 6; ++i) {
		cout << "arm.pf_def.inertia[" << i << "]: " << the_robot->ecp_command.instruction.arm.pf_def.inertia[i] << endl;
	}

	for (int i = 0; i < 6; ++i) {
		cout << "arm.pf_def.reciprocal_damping[" << i << "]: "
				<< the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[i] << endl;
	}

	for (int i = 0; i < 6; ++i) {
		cout << "arm.pf_def.force_xyz_torque_xyz[" << i << "]: "
				<< the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] << endl;
	}

	cout << "" << the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate << endl;

	cout << "get_arm_type " << the_robot->ecp_command.instruction.get_arm_type << endl;
	cout << "get_robot_model_type " << the_robot->ecp_command.instruction.get_robot_model_type << endl;
	cout << "get_type" << the_robot->ecp_command.instruction.get_type << endl;
	cout << "instruction_type" << the_robot->ecp_command.instruction.instruction_type << endl;
	cout << "interpolation_type " << the_robot->ecp_command.instruction.interpolation_type << endl;
	cout << "motion_steps " << the_robot->ecp_command.instruction.motion_steps << endl;
	cout << "motion_type " << the_robot->ecp_command.instruction.motion_type << endl;
	cout << "output_values " << the_robot->ecp_command.instruction.output_values << endl;

	cout << "robot_model.tool_frame_def.tool_frame "
			<< the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame << endl;
	cout << "robot_model.kinematic_model.kinematic_model_no "
			<< the_robot->ecp_command.instruction.robot_model.kinematic_model.kinematic_model_no << endl;

	for (int i = 0; i < MAX_SERVOS_NR; ++i) {
		cout << "robot_model.servo_algorithm.servo_algorithm_no [" << i << "]: "
				<< the_robot->ecp_command.instruction.robot_model.servo_algorithm.servo_algorithm_no[i] << endl;
	}

	for (int i = 0; i < MAX_SERVOS_NR; ++i) {
		cout << "robot_model.servo_algorithm.servo_parameters_no [" << i << "]: "
				<< the_robot->ecp_command.instruction.robot_model.servo_algorithm.servo_parameters_no[i] << endl;
	}

	for (int i = 0; i < 3; ++i) {
		cout << "robot_model.force_tool.position[" << i << "] "
				<< the_robot->ecp_command.instruction.robot_model.force_tool.position[i] << endl;
	}

	cout << "robot_model.force_tool.weight " << the_robot->ecp_command.instruction.robot_model.force_tool.weight
			<< endl;

	cout << "set_arm_type " << the_robot->ecp_command.instruction.set_arm_type << endl;
	cout << "set_robot_model_type " << the_robot->ecp_command.instruction.set_robot_model_type << endl;
	cout << "set_type " << the_robot->ecp_command.instruction.set_type << endl;
	cout << "value_in_step_no " << the_robot->ecp_command.instruction.value_in_step_no << endl;

	/*cout << "" << the_robot->ecp_command.instruction. << endl;
	 cout << "" << the_robot->ecp_command.instruction << endl;
	 cout << "" << the_robot->ecp_command.instruction << endl;
	 cout << "" << the_robot->ecp_command.instruction << endl;*/

	cout << "============================================================================================\n";
	cout << "ecp_g_mboryn::printInstruction(void) end \n";
	cout << cout.flush();
}

void ecp_g_mboryn::log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!logEnabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush(stdout);
	va_end(ap);
}

} // namespace mrrocpp

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

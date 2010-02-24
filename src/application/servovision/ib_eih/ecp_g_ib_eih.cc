/*
 * $Id: ecp_g_ib_eih.cc 3567 2010-01-13 20:42:58Z mboryn $
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_g_ib_eih.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 30

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace generator {

const char ecp_g_ib_eih::configSectionName[] = { "[servovision_ib_eih]" };

ecp_g_ib_eih::ecp_g_ib_eih(mrrocpp::ecp::common::task::task & _ecp_task)
//: mrrocpp::ecp::common::generator::generator(_ecp_task)
:
	generator(_ecp_task), logEnabled(true)
{
	char kp_name[] = { "kp" };
	char max_v_x_name[] = { "max_v_x" };
	char max_v_y_name[] = { "max_v_y" };
	char max_v_z_name[] = { "max_v_z" };
	char max_a_x_name[] = { "max_a_x" };
	char max_a_y_name[] = { "max_a_y" };
	char max_a_z_name[] = { "max_a_z" };

	if (_ecp_task.config.exists(std::string(kp_name), configSectionName)) {
		Kp = _ecp_task.config.value <double> (kp_name, configSectionName);
	} else {
		Kp = 0.0001;
		log("Parameter %s->%s not found. Using default value: %g\n", configSectionName, kp_name);
	}

	/*if (_ecp_task.config.exists(maxt_name, configSectionName)) {
	 maxT = _ecp_task.config.value <double> (maxt_name, configSectionName);
	 } else {
	 maxT = 0.01;
	 log("Parameter %s->%s not found. Using default value: %g\n", configSectionName, maxt_name, maxT);
	 }

	 log("\nKp: %g; maxT: %g\n", Kp, maxT);*/
	log("\nKp: %g\n", Kp);

	max_v[0]
			= _ecp_task.config.exists(std::string(max_v_x_name), configSectionName) ? _ecp_task.config.value <double> (max_v_x_name, configSectionName) : 0;
	max_v[1]
			= _ecp_task.config.exists(std::string(max_v_y_name), configSectionName) ? _ecp_task.config.value <double> (max_v_y_name, configSectionName) : 0;
	max_v[2]
			= _ecp_task.config.exists(std::string(max_v_z_name), configSectionName) ? _ecp_task.config.value <double> (max_v_z_name, configSectionName) : 0;

	max_a[0]
			= _ecp_task.config.exists(std::string(max_a_x_name), configSectionName) ? _ecp_task.config.value <double> (max_a_x_name, configSectionName) : 0;
	max_a[1]
			= _ecp_task.config.exists(std::string(max_a_y_name), configSectionName) ? _ecp_task.config.value <double> (max_a_y_name, configSectionName) : 0;
	max_a[2]
			= _ecp_task.config.exists(max_a_z_name, configSectionName) ? _ecp_task.config.value <double> (max_a_z_name, configSectionName) : 0;

	for (int i = 0; i < 3; ++i) {
		log("max_v[%d]: %g\t", i, max_v[i]);
	}
	log("\n");
	for (int i = 0; i < 3; ++i) {
		log("max_a[%d]: %g\t", i, max_a[i]);
	}
	log("\n");
}

ecp_g_ib_eih::~ecp_g_ib_eih()
{
}

bool ecp_g_ib_eih::first_step()
{
	log("ecp_g_ib_eih::first_step()\n");
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

	return true;
}

bool ecp_g_ib_eih::next_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	if (!currentFrameSaved) { // save first frame
		//log("ecp_g_ib_eih::next_step() 1\n");
		currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		currentGripperCoordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;

		currentFrameSaved = true;
	}

	// calculate translation
	lib::Xyz_Angle_Axis_vector l_vector;
	currentFrame.get_xyz_angle_axis(l_vector);

	if (vsp_fradia->from_vsp.vsp_report == lib::VSP_REPLY_OK) {
		//log("vsp_fradia->from_vsp.vsp_report == lib::VSP_REPLY_OK, %d, %d\n", vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z, (int)vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.tracking);
		if (vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.tracking) {

			mrrocpp::lib::K_vector
					e(vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.x, -vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.y, vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z); // error [pixels]

			u = e * Kp;
			u[2] = 0; // Z axis

			//for(int i=0; i<3; ++i){		log("e[%d] = %g\t", i, e[i]);	}		log("\n");
			//for(int i=0; i<3; ++i){		log("u[%d] = %g\t", i, u[i]);	}		log("\n");
			bool isSpeedConstrained[3] = { false, false, false };
			bool isAccelConstrained[3] = { false, false, false };
			for (int i = 0; i < 3; ++i) {
				// speed constraints
				if (u[i] > max_v[i]) {
					u[i] = max_v[i];
					isSpeedConstrained[i] = true;
				}
				if (u[i] < -max_v[i]) {
					u[i] = -max_v[i];
					isSpeedConstrained[i] = true;
				}

				// acceleration constraints
				/*if (u[i] - prev_u[i] > max_a[i]) {
					u[i] = prev_u[i] + max_a[i];
					isAccelConstrained[i] = true;
				}
				if (u[i] - prev_u[i] < -max_a[i]) {
					u[i] = prev_u[i] - max_a[i];
					isAccelConstrained[i] = true;
				}*/
				prev_u[i] = u[i];

				l_vector[i] += u[i]; // first 3 elements of l_vector[] are XYZ translation
			}
			//log("isSpeedConstrained[] = {%d, %d}\n", isSpeedConstrained[0], isSpeedConstrained[1]);

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
	}
	else{
		//log("!isArmFrameOk(nextFrame)\n");
	}
	currentFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = currentGripperCoordinate;

	//log("ecp_g_ib_eih::next_step() end\n");

	return true;
} // next_step()

bool ecp_g_ib_eih::isArmFrameOk(const lib::Homog_matrix& arm_frame)
{
	double minT[] = {0.600, -0.400, 0.200};
	double maxT[] = {0.950, 0.400, 0.300};
	lib::Xyz_Angle_Axis_vector l_vector;
	arm_frame.get_xyz_angle_axis(l_vector);

	for (int i = 0; i < 3; ++i) {
		if (l_vector[i] < minT[i] || l_vector[i] > maxT[i]) {
			//log("l_vector[%d] = %g not in range: %g ... %g\n", i, l_vector[i], minT[i], maxT[i]);
			return false;
		}
	}

	return true;
}

void ecp_g_ib_eih::log(const char *fmt, ...)
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

/*
 * $Id$
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_g_mboryn.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 25

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace generator {

const char ecp_g_mboryn::configSectionName[] = { "[mboryn_servovision]" };

ecp_g_mboryn::ecp_g_mboryn(mrrocpp::ecp::common::task::task & _ecp_task)
//: mrrocpp::ecp::common::generator::generator(_ecp_task)
:
	generator(_ecp_task)
{
	char kp_name[] = { "kp" };
	char maxt_name[] = { "maxt" };

	if (_ecp_task.config.exists(kp_name, configSectionName)) {
		Kp = _ecp_task.config.value <double> (kp_name, configSectionName);
	} else {
		Kp = 0.002;
		cout << "Parameter \"" << configSectionName << "\"->\"" << kp_name << "\" not found. Using default value: "
				<< Kp << endl;
	}

	if (_ecp_task.config.exists(maxt_name, configSectionName)) {
		maxT = _ecp_task.config.value <double> (maxt_name, configSectionName);
	} else {
		maxT = 0.001;
		cout << "Parameter \"" << configSectionName << "\"->\"" << maxt_name << "\" not found. Using default value: "
				<< maxT << endl;
	}

	printf("\nKp: %g; maxT: %g\n", Kp, maxT); fflush(stdout);
}

ecp_g_mboryn::~ecp_g_mboryn()
{
}

bool ecp_g_mboryn::first_step()
{
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 1;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	currentFrameSaved = false;

	return true;
}

#if 0

#else
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
		//printf("vsp_report == lib::VSP_REPLY_OK\n");		fflush(stdout);

		if (vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.tracking) {

			mrrocpp::lib::K_vector
					e(vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.x, -vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.y, vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z); // error [pixels]

			mrrocpp::lib::K_vector u;
			u = e * Kp;
			u[2] = 0; // Z axis
			for (int i = 0; i < 3; ++i) {
				if (u[i] > maxT) {
					cout << "u[" << i << "] > maxT: u[" << i << "] = " << u[i] << endl;
					u[i] = maxT;
				}
				if (u[i] < -maxT) {
					cout << "u[" << i << "] < -maxT: u[" << i << "] = " << u[i] << endl;
					u[i] = -maxT;
				}
				l_vector[i]+=u[i];
			}

			//translation += u;
		} else {
			//printf("Not tracking.\n");			fflush(stdout);
		}
	} else {
		//printf("vsp_report != lib::VSP_REPLY_OK\n");		fflush(stdout);
	}

	lib::Homog_matrix nextFrame;
	nextFrame.set_from_xyz_angle_axis(l_vector);

	// set next frame
	if (isArmFrameOk(nextFrame)) {
		nextFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	} else {
		currentFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	}
	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = currentGripperCoordinate;

	return true;
} // next_step()
#endif

bool ecp_g_mboryn::isArmFrameOk(const lib::Homog_matrix& arm_frame)
{
	lib::K_vector minT(0.590, -0.400, 0.200), maxT(1.000, 0.400, 0.300);
	lib::Xyz_Angle_Axis_vector l_vector;
	arm_frame.get_xyz_angle_axis(l_vector);
	//cout << "\naxis_with_angle: " << axis_with_angle << "translation: " << translation << endl;

	for (int i = 0; i < 3; ++i) {
		if (l_vector[i] < minT[i] || l_vector[i] > maxT[i]) {
			return false;
		}
	}

	return true;
}

void ecp_g_mboryn::log(char *fmt, ...)
{

}

} // namespace mrrocpp

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

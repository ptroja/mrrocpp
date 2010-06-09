/*
 * ecp_g_pb_eih.cc
 *
 *  Created on: Mar 18, 2010
 *      Author: mboryn
 */

#include "ecp_g_pb_eih.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

const char ecp_g_pb_eih::configSectionName[] = { "[servovision_pb_eih]" };

ecp_g_pb_eih::ecp_g_pb_eih(mrrocpp::ecp::common::task::task & _ecp_task, ecp_mp::sensor::fradia_sensor <
		pb_object_position> *vsp_fradia, visual_servo_regulator <4, 4> * regulator) :
	visual_servo <4, 4> (_ecp_task, regulator), vsp_fradia(vsp_fradia), delta_t(MOTION_STEPS * 2e-3)
{
	sensor_m[lib::SENSOR_CVFRADIA] = vsp_fradia;

	max_v = _ecp_task.config.value <double> ("max_v", configSectionName);
	max_a = _ecp_task.config.value <double> ("max_a", configSectionName);

	Eigen::Matrix <double, 3, 4> e_T_c;
	e_T_c = _ecp_task.config.value <3, 4> ("e_t_c_frame", configSectionName);

//	double rot[3][3];

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 4; ++j) {
//			rot[i][j] = e_T_c(i, j);
			e_T_cFrame[i][j] = e_T_c(i, j);
		}
	}

//	e_T_cFrame.set_rotation_matrix(rot);
//	e_T_cFrame.set_translation_vector(e_T_c(0, 3), e_T_c(1, 3), e_T_c(2, 3));

	if (logEnabled) {
		log("max_v: %g\n", max_v);

		log("max_a: %g\n", max_a);
		cout << "e_T_cFrame: \n" << e_T_cFrame << "\n\n";
	}

}

ecp_g_pb_eih::~ecp_g_pb_eih()
{

}

bool ecp_g_pb_eih::first_step()
{
	logDbg("ecp_g_ib_eih::first_step()\n");

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

bool ecp_g_pb_eih::next_step()
{
	logDbg("ecp_g_ib_eih::next_step() begin\n");
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	if (!currentFrameSaved) { // save first frame
		//logDbg("ecp_g_ib_eih::next_step() 1\n");
		currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		currentGripperCoordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;

		currentFrameSaved = true;
	}

	logDbg("ecp_g_ib_eih::next_step() 1\n");

	lib::Homog_matrix nextFrame, deltaFrame;

	lib::K_vector u_translation;
	lib::Homog_matrix u_rotation;

	logDbg("ecp_g_ib_eih::next_step() 2\n");

	if (vsp_fradia->image.size == sizeof(visual_object_tracker_t) && vsp_fradia->image.tracking) {
		logDbg("ecp_g_ib_eih::next_step() 3\n");
		Eigen::Matrix <double, 4, 1> e;
		e(0, 0) = vsp_fradia->image.x;
		e(1, 0) = vsp_fradia->image.y;
		//e(2, 0) = vsp_fradia->received_object.z;
		e(2, 0) = 0;
		e(3, 0) = vsp_fradia->image.alpha;

		log("ecp_g_ib_eih::next_step() vsp_fradia->received_object.x: %g\n", (double) vsp_fradia->image.x);
		log("ecp_g_ib_eih::next_step() vsp_fradia->received_object.y: %g\n", (double) vsp_fradia->image.y);
		log("ecp_g_ib_eih::next_step() vsp_fradia->received_object.z: %g\n", (double) vsp_fradia->image.z);
		log("ecp_g_ib_eih::next_step() vsp_fradia->received_object.alpha: %g\n", vsp_fradia->image.alpha);

		logDbg("ecp_g_ib_eih::next_step() 4\n");

		Eigen::Matrix <double, 4, 1> control;

		logDbg("ecp_g_ib_eih::next_step() 4a\n");

		control = regulator->calculate_control(e);

		logDbg("ecp_g_ib_eih::next_step() 5\n");

		Eigen::Matrix <double, 3, 1> camera_to_object_translation;
		camera_to_object_translation(0, 0) = control(0, 0);
		camera_to_object_translation(1, 0) = control(1, 0);
		camera_to_object_translation(2, 0) = control(2, 0);

		logDbg("ecp_g_ib_eih::next_step() 6\n");

		u_translation = e_T_cFrame * camera_to_object_translation;

		logDbg("ecp_g_ib_eih::next_step() 7\n");

		logDbg("e[3] = %g\t", e(3, 0));
		logDbg("control [3] = %g\n", control(3, 0));

		u_rotation.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0, 0, 0, 0, 0, control(3, 0)));

		logDbg("ecp_g_ib_eih::next_step() 9\n");
		for (int i = 0; i < 4; ++i) {
			logDbg("e[%d] = %g\t", i, e(i, 0));
		}
		logDbg("\n");
		for (int i = 0; i < 3; ++i) {
			logDbg("u_translation[%d] = %g\t", i, u_translation(i, 0));
		}
		logDbg("\n");

		//logDbg("Tracking.\n");
	} else if (vsp_fradia->image.size != sizeof(visual_object_tracker_t)) {
		log("Size of received structure doesn't match it's size field.\n");
	}

	logDbg("ecp_g_ib_eih::next_step() 10\n");
	// speed constraints

	double ds = u_translation.squaredNorm();
	if (ds > (max_v * delta_t)) {
		u_translation = u_translation * ((max_v * delta_t) / ds);
		//logDbg("Speed constrained (%g > %g).\n", ds, (max_v * delta_t));
	}

	logDbg("ecp_g_ib_eih::next_step() 11\n");

	// acceleration constraints
	Eigen::Matrix <double, 3, 1> du = u_translation - prev_u;
	double d2s = du.squaredNorm();
	if (d2s > (max_a * delta_t * delta_t)) {
		u_translation = prev_u + (u_translation - prev_u) * ((max_a * delta_t * delta_t) / d2s);
		//logDbg("Acceleration constrained (%g > %g).\n", d2s, (max_a * delta_t * delta_t));
	}

	logDbg("ecp_g_ib_eih::next_step() 12\n");

	//deltaFrame.set_rotation_matrix(u_rotation);
	deltaFrame.set_translation_vector(u_translation);

	nextFrame = currentFrame * deltaFrame;

	cout << "nextFrame:\n" << nextFrame << "\n\n";
	cout << "currentFrame:\n" << currentFrame << "\n\n";
	cout << "e_T_cFrame:\n" << e_T_cFrame << "\n\n";
	cout << "deltaFrame:\n" << deltaFrame << "\n\n";

	// set next frame

	logDbg("ecp_g_ib_eih::next_step() 13\n");

	if (isArmFrameOk(nextFrame)) {
		currentFrame = nextFrame;
	} else {
		//log("!isArmFrameOk(nextFrame)\n");
	}

	logDbg("ecp_g_ib_eih::next_step() 14\n");

	currentFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = currentGripperCoordinate;

	prev_u = u_translation;

	logDbg("ecp_g_ib_eih::next_step() end\n");

	return true;
} // next_step()

bool ecp_g_pb_eih::isArmFrameOk(const lib::Homog_matrix& arm_frame)
{
	double minT[] = { 0.600, -0.400, 0.150 };
	double maxT[] = { 0.950, 0.400, 0.300 };
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

} // namespace generator

}

}

}

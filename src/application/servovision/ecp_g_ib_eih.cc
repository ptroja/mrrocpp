/*
 * $Id$
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

namespace common {

namespace generator {

const char ecp_g_ib_eih::configSectionName[] = { "[servovision_ib_eih]" };

ecp_g_ib_eih::ecp_g_ib_eih(mrrocpp::ecp::common::task::task & _ecp_task, ecp_mp::sensor::fradia_sensor <object_tracker> *vsp_fradia, visual_servo_regulator * regulator) :
	visual_servo(_ecp_task, regulator), vsp_fradia(vsp_fradia), prev_u(3), delta_t(MOTION_STEPS * 2e-3)
{
	sensor_m[lib::SENSOR_CVFRADIA] = vsp_fradia;

	max_v = _ecp_task.config.value <double> ("max_v", configSectionName);
	max_a = _ecp_task.config.value <double> ("max_a", configSectionName);

	log("max_v: %g\n", max_v);

	log("max_a: %g\n", max_a);

	e_T_cFrame.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0, 0, 0, 0, 0, M_PI ));
}

ecp_g_ib_eih::~ecp_g_ib_eih()
{
}

bool ecp_g_ib_eih::first_step()
{
	log("ecp_g_ib_eih::first_step()\n");

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
	//log("ecp_g_ib_eih::next_step() begin\n"); fflush(stdout);
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	if (!currentFrameSaved) { // save first frame
		//log("ecp_g_ib_eih::next_step() 1\n");
		currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		currentGripperCoordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;

		currentFrameSaved = true;
	}

	lib::Homog_matrix nextFrame, deltaFrame;

	//boost::numeric::ublas::vector<double> u(3);
	lib::K_vector u_translation;

	if (vsp_fradia->received_object.tracking) {
		boost::numeric::ublas::vector<double> e(3);
		e(0) = vsp_fradia->received_object.x;
		e(1) = vsp_fradia->received_object.y;
		e(2) = vsp_fradia->received_object.z;

		boost::numeric::ublas::vector<double> u_c(3);
		u_c = regulator->calculate_control(e);

		u_c[2] = 0; // Z axis

		u_translation = e_T_cFrame * lib::K_vector(u_c(0), u_c(1), u_c(2));

		//for(int i=0; i<3; ++i){		log("e[%d] = %g\t", i, e[i]);	}		log("\n");
		//for(int i=0; i<3; ++i){		log("u[%d] = %g\t", i, u[i]);	}		log("\n");

		//log("Tracking.\n");
	} else {
		//log("Not tracking.\n");
	}

	// speed constraints
	double ds = boost::numeric::ublas::norm_2(u_translation);
	if (ds > (max_v * delta_t)) {
		u_translation = u_translation * ((max_v * delta_t) / ds);
		//log("Speed constrained (%g > %g).\n", ds, (max_v * delta_t));
	}

	// acceleration constraints
	double d2s = boost::numeric::ublas::norm_2(u_translation - prev_u);
	if (d2s > (max_a * delta_t * delta_t)) {
		u_translation = prev_u + (u_translation - prev_u) * ((max_a * delta_t * delta_t) / d2s);
		//log("Acceleration constrained (%g > %g).\n", d2s, (max_a * delta_t * delta_t));
	}

	deltaFrame.set_translation_vector(u_translation);

	nextFrame = currentFrame * deltaFrame;

	/*cout<<"nextFrame:\n"<<nextFrame<<endl;
	cout<<"currentFrame:\n"<<currentFrame<<endl;
	cout<<"e_T_cFrame:\n"<<e_T_cFrame<<endl;
	cout<<"deltaFrame:\n"<<deltaFrame<<endl;*/

	// set next frame
	if (isArmFrameOk(nextFrame)) {
		currentFrame = nextFrame;
	} else {
		//log("!isArmFrameOk(nextFrame)\n");
	}
	currentFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = currentGripperCoordinate;

	//log("ecp_g_ib_eih::next_step() end\n"); fflush(stdout);

	prev_u = u_translation;

	return true;
} // next_step()

bool ecp_g_ib_eih::isArmFrameOk(const lib::Homog_matrix& arm_frame)
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

} // namespace mrrocpp

} // namespace common

} // namespace ecp

} // namespace mrrocpp

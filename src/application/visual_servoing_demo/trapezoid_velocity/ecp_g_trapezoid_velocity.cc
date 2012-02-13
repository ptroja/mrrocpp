/*
 * trapezoid_velocity.cc
 *
 *  Created on: 25-02-2011
 *      Author: mateusz
 */

#include "ecp_g_trapezoid_velocity.h"

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "base/lib/logger.h"

using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

trapezoid_velocity::trapezoid_velocity(mrrocpp::ecp::common::task::task & ecp_task) :
		common::generator::generator(ecp_task), motion_steps(30)
{
}

trapezoid_velocity::~trapezoid_velocity()
{
	// TODO Auto-generated destructor stub
}

void trapezoid_velocity::set_params(int axis_idx, double accel1, double accel2, double v_max)
{
	this->axis_idx = axis_idx;
	this->accel1 = accel1;
	this->accel2 = accel2;
	this->v_max = v_max;
}

bool trapezoid_velocity::first_step()
{
	log_dbg("trapezoid_generator::first_step() begin\n");

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;

	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::JOINT;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = motion_steps;
	the_robot->ecp_command.value_in_step_no = motion_steps - 3;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	current_theta_saved = false;
	state = S_INIT;
	steps_count = 0;
	dt = motion_steps * 2e-3;

	log_dbg("trapezoid_generator::first_step() end\n");
	return true;
}

bool trapezoid_velocity::next_step()
{
	if (!current_theta_saved) { // save first position
		for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i) {
			current_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.joint_coordinates[i];
		}
		s = current_arm_coordinates[axis_idx];
		v = 0;
		current_theta_saved = true;
	}

	bool continueMotion = true;
	if (state == S_INIT) {
		steps_count++;
		if (steps_count >= STEPS_NUMBER_INIT_STOP) {
			steps_count = 0;
			state = S_ACCEL;
		}
	} else if (state == S_ACCEL) {
		double dv = accel1 * dt;
		v += dv;
		if (v >= v_max) {
			v = v_max;
			steps_count = 0;
			state = S_CONST_SPEED;
		}
		double ds = v * dt;
		s += ds;
	} else if (state == S_CONST_SPEED) {
		double ds = v * dt;
		s += ds;

		steps_count++;
		if (steps_count >= STEPS_NUMBER_CONST_SPEED) {
			steps_count = 0;
			state = S_SLOWDOWN;
		}
	} else if (state == S_SLOWDOWN) {
		double dv = -accel2 * dt;
		v += dv;
		if (v <= 0) {
			v = 0;
			steps_count = 0;
			state = S_STOP;
		}
		double ds = v * dt;
		s += ds;
	} else if (state == S_STOP) {
		steps_count++;
		if (steps_count >= STEPS_NUMBER_INIT_STOP) {
			steps_count = 0;
			//state = S_ACCEL;
			continueMotion = false;
		}
	}

	// prepare command to EDP
	current_arm_coordinates[axis_idx] = s;
	the_robot->ecp_command.instruction_type = lib::SET_GET;
	for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = current_arm_coordinates[i];
	}
	return continueMotion;
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

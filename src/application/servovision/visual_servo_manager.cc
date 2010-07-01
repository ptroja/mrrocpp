/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "visual_servo_manager.h"

using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

visual_servo_manager::visual_servo_manager(
		mrrocpp::ecp::common::task::task & ecp_task,
		const std::string& section_name) :
	generator(ecp_task), current_position_saved(false), motion_steps(30),
			a_max(0), v_max(0) {
	// 2 ms per one step
	dt = motion_steps * 0.002;

	v_max = ecp_task.config.value<double> ("v_max", section_name);
	a_max = ecp_task.config.value<double> ("a_max", section_name);

	//	log("a_max: %lg, v_max: %lg\n", a_max, v_max);
	//	log("v_max * dt: %lg\n", v_max * dt);
}

visual_servo_manager::~visual_servo_manager() {
}

bool visual_servo_manager::first_step()
{
	log_dbg("ecp_g_ib_eih::first_step()\n");

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = motion_steps;
	the_robot->ecp_command.instruction.value_in_step_no = motion_steps - 3;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i]
				= lib::UNGUARDED_MOTION;
	}

	current_position_saved = false;

	//	sigevent ev;
	//	SIGEV_NONE_INIT(&ev);
	//
	//	if (timer_create(CLOCK_REALTIME, &ev, &timerek) < 0) {
	//		log("timer_create(CLOCK_REALTIME, ev, timerek) < 0: %d\n", errno);
	//	}
	//
	//	setup_timer();

	prev_v.setZero();
	v.setZero();
	a.setZero();

	for (int i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->reset();
	}

	return true;
}

//void visual_servo_manager::setup_timer()
//{
//	max_t.it_value.tv_sec = 1;
//	max_t.it_value.tv_nsec = 0;
//	timer_settime(timerek, 0, &max_t, NULL);
//}

bool visual_servo_manager::next_step() {
	//	timer_gettime(timerek, &curr_t);
	//	log("timer_gettime(timerek, &curr_t): %d.%09d\n", curr_t.it_value.tv_sec, curr_t.it_value.tv_nsec);
	//	setup_timer();

	//	log_dbg("bool visual_servo_manager::next_step() begin\n");
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	if (!current_position_saved) { // save first frame
		//log_dbg("ecp_g_ib_eih::next_step() 1\n");
		current_position.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);

		current_position_saved = true;
	}

	bool object_visible = false;
	for (std::vector<boost::shared_ptr<visual_servo> >::iterator it =
			servos.begin(); it != servos.end(); ++it) {
		(*it)->get_vsp_fradia()->get_reading();
		object_visible = object_visible || (*it)->is_object_visible();
	}

	// update object visiblity in termination conditions
	for (int i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->update_object_visibility(object_visible);
	}

	// get readings from all servos and aggregate
	lib::Homog_matrix position_change = get_aggregated_position_change();
	//	log_dbg("bool visual_servo_manager::next_step(): position_change = (%+07.3lg, %+07.3lg, %+07.3lg)\n", position_change(0, 3), position_change(1, 3), position_change(2, 3));

	lib::Homog_matrix next_position = current_position * position_change;

	//	log_dbg("bool visual_servo_manager::next_step(): next_position = (%+07.3lg, %+07.3lg, %+07.3lg)\n", next_position(0, 3), next_position(1, 3), next_position(2, 3));

	// apply weak position constraints
	bool constraints_kept = false;
	for (int i = 0; i < position_constraints.size(); ++i) {
		if (position_constraints[i]->is_position_ok(next_position)) {
			constraints_kept = true;
		}
	}
	if (!constraints_kept && position_constraints.size() > 0) {
		position_constraints[0]->apply_constraint(next_position);
	}

	apply_speed_accel_constraints(next_position);

	// update speed and acceleration in termination conditions
	for (int i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->update_end_effector_speed(v);
		termination_conditions[i]->update_end_effector_accel(a);
	}

	//	log_dbg("bool visual_servo_manager::next_step(): next_position = (%+07.3lg, %+07.3lg, %+07.3lg)\n", next_position(0, 3), next_position(1, 3), next_position(2, 3));
	// send command to the robot
	next_position.get_frame_tab(
			the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

	// save next position
	current_position = next_position;

	// check termination conditions
	for (int i = 0; i < termination_conditions.size(); ++i) {
		if (termination_conditions[i]->terminate_now()) {
			return false;
		}
	}

	return true;
} // next_step()

void visual_servo_manager::apply_speed_accel_constraints(
		lib::Homog_matrix& new_position) {
	// apply speed constraints
	Eigen::Matrix<double, 3, 1> ds;

	for (int i = 0; i < 3; ++i) {
		ds(i, 0) = new_position(i, 3) - current_position(i, 3);
	}

	double ds_norm = ds.norm();
	if (ds_norm > (v_max * dt)) {
		ds = ds * ((v_max * dt) / ds_norm);
	}

	v = ds / dt;

	// apply acceleration constraints
	Eigen::Matrix<double, 3, 1> dv = v - prev_v;
	double dv_norm = dv.norm();
	//	log_dbg("dv = %10lg    prev_v = %10lg     v = %10lg\n", dv_norm, prev_v.norm(), v.norm());
	if (dv_norm > (a_max * dt)) {
		dv = dv * ((a_max * dt) / dv_norm);
		v = prev_v + dv;
		ds = v * dt;
		//		log_dbg("Acceleration constrained (%10lg > %10lg): dv = %10lg  ds = %10lg\n", dv_norm, (a_max * dt), dv.norm(), ds.norm());
	}
	//	log_dbg("ds = %10lg\n", ds.norm());

	for (int i = 0; i < 3; ++i) {
		new_position(i, 3) = current_position(i, 3) + ds(i, 0);
	}

	a = dv / dt;
	prev_v = v;
}

const lib::Homog_matrix& visual_servo_manager::get_current_position() const {
	return current_position;
}

void visual_servo_manager::configure()
{
	//	log_dbg("void visual_servo_manager::configure() 1\n");
	configure_all_servos();
	//	log_dbg("void visual_servo_manager::configure() 2\n");
	for (std::vector <boost::shared_ptr <visual_servo> >::iterator it = servos.begin(); it != servos.end(); ++it) {
		(*it)->get_vsp_fradia()->configure_sensor();
	}
	//	log_dbg("void visual_servo_manager::configure() 3\n");
}

void visual_servo_manager::add_position_constraint(boost::shared_ptr<
		position_constraint> new_constraint) {
	position_constraints.push_back(new_constraint);
}

void visual_servo_manager::add_termination_condition(boost::shared_ptr<
		termination_condition> term_cond) {
	termination_conditions.push_back(term_cond);
}

void visual_servo_manager::set_speed_accel_constraints(double v_max,
		double a_max) {
	this->a_max = a_max;
	this->v_max = v_max;
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

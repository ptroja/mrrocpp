/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"

#include "visual_servo_manager.h"

using namespace logger;
using namespace mrrocpp::ecp::servovision;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

visual_servo_manager::visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
		common::generator::generator(ecp_task), current_position_saved(false), motion_steps(30), max_speed(0), max_angular_speed(0),
			max_acceleration(0), max_angular_acceleration(0)
{
	// 2 ms per one step
	dt = motion_steps * 0.002;

	max_speed = ecp_task.config.value <double> ("v_max", section_name);
	max_angular_speed = ecp_task.config.value <double> ("omega_max", section_name);
	max_acceleration = ecp_task.config.value <double> ("a_max", section_name);
	max_angular_acceleration = ecp_task.config.value <double> ("epsilon_max", section_name);

	//	log("a_max: %lg, v_max: %lg\n", a_max, v_max);
	//	log("v_max * dt: %lg\n", v_max * dt);
}

visual_servo_manager::~visual_servo_manager()
{
}

bool visual_servo_manager::first_step()
{
	log_dbg("visual_servo_manager::first_step() begin\n");

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = motion_steps;
	the_robot->ecp_command.value_in_step_no = motion_steps - 3;
	log_dbg("visual_servo_manager::first_step() begin1\n");
	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	current_position_saved = false;
	log_dbg("visual_servo_manager::first_step() begin2\n");
	//	sigevent ev;
	//	SIGEV_NONE_INIT(&ev);
	//
	//	if (timer_create(CLOCK_REALTIME, &ev, &timerek) < 0) {
	//		log("timer_create(CLOCK_REALTIME, ev, timerek) < 0: %d\n", errno);
	//	}
	//
	//	setup_timer();

	prev_velocity.setZero();
	prev_angular_velocity.setZero();
	velocity.setZero();
	angular_velocity.setZero();
	acceleration.setZero();
	angular_acceleration.setZero();
	log_dbg("visual_servo_manager::first_step() begin3\n");
	for (size_t i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->reset();
	}
	log_dbg("visual_servo_manager::first_step() begin4\n");
	return true;
}

//void visual_servo_manager::setup_timer()
//{
//	max_t.it_value.tv_sec = 1;
//	max_t.it_value.tv_nsec = 0;
//	timer_settime(timerek, 0, &max_t, NULL);
//}

//	timer_gettime(timerek, &curr_t);
//	log("timer_gettime(timerek, &curr_t): %d.%09d\n", curr_t.it_value.tv_sec, curr_t.it_value.tv_nsec);
//	setup_timer();

bool visual_servo_manager::next_step()
{
	if (!current_position_saved) { // save first position
		current_position.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		current_position_saved = true;
	}

	// get aggregated position change from all servos
	lib::Homog_matrix position_change = get_aggregated_position_change();

	// calculate new position with respect to robot's base
	lib::Homog_matrix next_position = current_position * position_change;

	// apply weak position constraints
	constrain_position(next_position);

	// position change is now different, because position constraints were applied
	position_change = (!current_position) * next_position;

	// change representation to AA, because in AA form it's easier to constrain velocity and acceleration
	constrain_speed_accel(position_change);

	// update next position, because it may have changed by velocity and acceleration constraints
	next_position = current_position * position_change;

	// prepare command to EDP
	the_robot->ecp_command.instruction_type = lib::SET_GET;
	next_position.get_frame_tab(the_robot->ecp_command.arm.pf_def.arm_frame);

	// save next position
	current_position = next_position;

	// check termination conditions
	bool any_condition_met = false;
	for (size_t i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->update(this);
		if (termination_conditions[i]->is_condition_met()) {
			any_condition_met = true;
		}
	}

	return !any_condition_met;
} // next_step()

void visual_servo_manager::constrain_position(lib::Homog_matrix & new_position)
{
	double nearest_allowed_area_distance = INFINITY;
	lib::Homog_matrix constrained_position = new_position;

	for (size_t i = 0; i < position_constraints.size(); ++i) {
		lib::Homog_matrix c1 = position_constraints[i]->apply_constraint(new_position);
		Eigen::Matrix <double, 3, 1> translation;
		translation(0, 0) = c1(0, 3) - new_position(0, 3);
		translation(1, 0) = c1(1, 3) - new_position(1, 3);
		translation(2, 0) = c1(2, 3) - new_position(2, 3);

		double d = translation.norm();

		if(nearest_allowed_area_distance > d){
			nearest_allowed_area_distance = d;
			constrained_position = c1;
		}
	}
	new_position = constrained_position;
}

void visual_servo_manager::constrain_vector(Eigen::Matrix <double, 3, 1> &ds, Eigen::Matrix <double, 3, 1> &prev_v, Eigen::Matrix <
		double, 3, 1> &v, Eigen::Matrix <double, 3, 1> &a, double max_v, double max_a)
{
	// apply speed constraints
	double ds_norm = ds.norm();
	if (ds_norm > (max_v * dt)) {
		ds = ds * ((max_v * dt) / ds_norm);
	}

	v = ds / dt;

	// apply acceleration constraints
	Eigen::Matrix <double, 3, 1> dv = v - prev_v;
	double dv_norm = dv.norm();
	if (dv_norm > (max_a * dt)) {
		dv = dv * ((max_a * dt) / dv_norm);
		v = prev_v + dv;
		ds = v * dt;
	}

	a = dv / dt;

	prev_v = v;
}

void visual_servo_manager::constrain_speed_accel(lib::Homog_matrix & position_change)
{
	lib::Xyz_Angle_Axis_vector aa_vector;
	position_change.get_xyz_angle_axis(aa_vector);
	Eigen::Matrix <double, 3, 1> ds = aa_vector.block(0, 0, 3, 1);
	Eigen::Matrix <double, 3, 1> dalpha = aa_vector.block(3, 0, 3, 1);

	constrain_vector(ds, prev_velocity, velocity, acceleration, max_speed, max_acceleration);
	constrain_vector(dalpha, prev_angular_velocity, angular_velocity, angular_acceleration, max_angular_speed, max_angular_acceleration);

	aa_vector.block(0, 0, 3, 1) = ds;
	aa_vector.block(3, 0, 3, 1) = dalpha;
	// get back to homogeneous matrix representation
	position_change.set_from_xyz_angle_axis(aa_vector);
}

const lib::Homog_matrix& visual_servo_manager::get_current_position() const
{
	return current_position;
}

void visual_servo_manager::configure(const std::string & sensor_prefix)
{
	//	log_dbg("void visual_servo_manager::configure() 1\n");
	configure_all_servos();
	//	log_dbg("void visual_servo_manager::configure() 2\n");
	int i = 0;
	for (std::vector <boost::shared_ptr <visual_servo> >::iterator it = servos.begin(); it != servos.end(); ++it, ++i) {
		(*it)->get_sensor()->configure_sensor();
		char sensor_suffix[64];
		sprintf(sensor_suffix, "%02d", i);
		lib::sensor::SENSOR_t sensor_id = sensor_prefix + sensor_suffix;
		sensor_m[sensor_id] = (*it)->get_sensor().get();
	}
	//	log_dbg("void visual_servo_manager::configure() 3\n");
}

void visual_servo_manager::add_position_constraint(boost::shared_ptr <position_constraint> new_constraint)
{
	position_constraints.push_back(new_constraint);
}

void visual_servo_manager::add_termination_condition(boost::shared_ptr <termination_condition> term_cond)
{
	termination_conditions.push_back(term_cond);
}

double visual_servo_manager::get_linear_speed() const
{
	return velocity.norm();
}

double visual_servo_manager::get_angular_speed() const
{
	return angular_velocity.norm();
}

double visual_servo_manager::get_linear_acceleration() const
{
	return acceleration.norm();
}

double visual_servo_manager::get_angular_acceleration() const
{
	return angular_acceleration.norm();
}

const std::vector <boost::shared_ptr <mrrocpp::ecp::servovision::visual_servo> >& visual_servo_manager::get_servos() const
{
	return servos;
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

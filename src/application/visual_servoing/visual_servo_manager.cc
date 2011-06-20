/*
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include <algorithm>

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"

#include "visual_servo_manager.h"

using namespace logger;
using namespace mrrocpp::ecp::servovision;
using namespace std;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

const int visual_servo_manager::motion_steps_default = 30;
const int visual_servo_manager::motion_steps_min = 10;
const int visual_servo_manager::motion_steps_max = 60;
//const int visual_servo_manager::motion_steps_value_in_step_no = 4;
const double visual_servo_manager::step_time = 0.002;

visual_servo_manager::visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
	common::generator::generator(ecp_task), current_position_saved(false), max_speed(0), max_angular_speed(0),
			max_acceleration(0), max_angular_acceleration(0)
{
	new_motion_steps = motion_steps = motion_steps_base
			= ecp_task.config.exists("motion_steps", section_name) ? ecp_task.config.value <unsigned int> ("motion_steps", section_name) : motion_steps_default;

	dt = motion_steps * step_time;

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
	new_motion_steps = motion_steps = motion_steps_base;
	value_in_step_no = motion_steps_base - 4;

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = motion_steps;
	//the_robot->ecp_command.value_in_step_no = motion_steps - motion_steps_value_in_step_no;
	the_robot->ecp_command.value_in_step_no = value_in_step_no;
	dt = motion_steps * step_time;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	current_position_saved = false;

	prev_velocity.setZero();
	prev_angular_velocity.setZero();
	velocity.setZero();
	angular_velocity.setZero();
	acceleration.setZero();
	angular_acceleration.setZero();

	for (size_t i = 0; i < termination_conditions.size(); ++i) {
		termination_conditions[i]->reset();
	}
	log_dbg("visual_servo_manager::first_step() end\n");

//	clock_gettime(CLOCK_REALTIME, &prev_timestamp);
//	c = 0;
//	ss << "motion_steps = " << motion_steps <<"\n";

	return true;
}

bool visual_servo_manager::next_step()
{
//	clock_gettime(CLOCK_REALTIME, &current_timestamp);
//	int sec = current_timestamp.tv_sec - prev_timestamp.tv_sec;
//	int nsec = current_timestamp.tv_nsec - prev_timestamp.tv_nsec;
//
//	double next_step_time = sec + 1e-9*nsec;
//
//	ss << "\nnext_step_time = " << next_step_time << "\n";
//	if(next_step_time < 0.005){
//		ss << "----------------------------------------- next_step_time < 0.005\n";
//	}
//
//	if(++c > 200){
//		c = 0;
//		cout<<"====================\n"<<ss.str()<<"=================\n";
//		ss.str("");
//	}
//
//	prev_timestamp = current_timestamp;

	if (!current_position_saved) { // save first position
		current_position = the_robot->reply_package.arm.pf_def.arm_frame;
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

	// prepare command to EDP
	motion_steps = new_motion_steps;

//	log_dbg("motion_steps = %d\n", motion_steps);

	the_robot->ecp_command.instruction_type = lib::SET_GET;
	the_robot->ecp_command.arm.pf_def.arm_frame = next_position;
	the_robot->ecp_command.motion_steps = motion_steps;
	//the_robot->ecp_command.value_in_step_no = motion_steps - motion_steps_value_in_step_no;
	the_robot->ecp_command.value_in_step_no = value_in_step_no;

	dt = motion_steps * step_time;

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

		if (nearest_allowed_area_distance > d) {
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

double visual_servo_manager::get_dt() const
{
	return dt;
}

void visual_servo_manager::set_new_motion_steps(int new_motion_steps)
{
//	ss<<"new_motion_steps = "<<new_motion_steps<<"\n";
	this->new_motion_steps = min(new_motion_steps, motion_steps_max);
	this->new_motion_steps = max(new_motion_steps, motion_steps_min);
	//log_dbg("visual_servo_manager::set_new_motion_steps(): this->new_motion_steps = %d\n", this->new_motion_steps);
}

int visual_servo_manager::get_new_motion_steps() const
{
	return new_motion_steps;
}

int visual_servo_manager::get_motion_steps() const
{
	return motion_steps;
}

int visual_servo_manager::get_motion_steps_base() const{
	return motion_steps_base;
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

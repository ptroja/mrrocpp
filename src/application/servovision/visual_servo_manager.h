/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_MANAGER_H_
#define VISUAL_SERVO_MANAGER_H_

#include <vector>
#include <boost/shared_ptr.hpp>

#include "ecp/common/generator/ecp_generator.h"
#include "visual_servo.h"
#include "position_constraint.h"
#include "termination_condition.h"

#include <signal.h>
#include <time.h>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 * Base generator for all visual servo managers.
 * Servo manager is a generator that takes control calculated in multiple visual_servo objects and aggregates them.
 * It also applies constraints for end effector speed, acceleration and position.
 */
class visual_servo_manager : public mrrocpp::ecp::common::generator::generator
{
public:
	virtual ~visual_servo_manager();

	virtual bool first_step();
	virtual bool next_step();

	/** Configures all servos. Calls configure_all_servos() of derived class and then for all sensors calls configure_sensor(). */
	virtual void configure();

	/**
	 * Add position constraint.
	 * At least one position constraint must be met to move end effector.
	 * If there are no position constraints, end effector position will be unconstrained,
	 * so it is likely to move end effector beyond it's limits.
	 * @param new_constraint
	 */
	void add_position_constraint(boost::shared_ptr <position_constraint> new_constraint);

	/**
	 * Add termination condition.
	 * Every next_step() all termination conditions are updated and checked. If at least one of them
	 * is met, next_step() will return false and generator will terminate.
	 * If there are no termination conditions, generator will never stop.
	 * @param term_cond
	 */
	void add_termination_condition(boost::shared_ptr <termination_condition> term_cond);

	void set_speed_accel_constraints(double v_max, double a_max);
protected:
	visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name);
	/**
	 * Called in next_step() to get aggregated control to pass to EDP.
	 * @return
	 */
	virtual lib::Homog_matrix get_aggregated_position_change() = 0;
	/**
	 * Called from constructor to initialize all servos. After this call, servos field must be initialized.
	 */
	virtual void configure_all_servos() = 0;
	std::vector <boost::shared_ptr <visual_servo> > servos;
	const lib::Homog_matrix& get_current_position() const;

	/** Time between next_step() calls */
	double dt;
private:
	lib::Homog_matrix current_position;
	bool current_position_saved;
	int motion_steps;
	double current_gripper_coordinate;

	std::vector <boost::shared_ptr <position_constraint> > position_constraints;
	std::vector <boost::shared_ptr <termination_condition> > termination_conditions;

	double a_max, v_max;
	/** Current end effector speed */
	Eigen::Matrix <double, 3, 1> v;

	/** Previous end effector speed */
	Eigen::Matrix <double, 3, 1> prev_v;

	/** End effector acceleration */
	Eigen::Matrix <double, 3, 1> a;

	/**
	 * Apply constraints for speed and acceleration.
	 * @param new_position Matrix will be modified to satisfy constraints.
	 */
	void apply_speed_accel_constraints(lib::Homog_matrix& new_position);

	//	timer_t timerek;
	//	itimerspec max_t;
	//	itimerspec curr_t;
	//
	//	void setup_timer();
};

/** @} */

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* VISUAL_SERVO_MANAGER_H_ */

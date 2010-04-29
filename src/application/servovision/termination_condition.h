/*
 * termination_condition.h
 *
 *  Created on: Apr 29, 2010
 *      Author: mboryn
 */

#ifndef TERMINATION_CONDITION_H_
#define TERMINATION_CONDITION_H_

#include <Eigen/Core>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 * Abstract class representing termination condition of visual_servo_manager generator.
 * One method must be implemented: terminate_now(), which is called for every termination condition in next_step() of visual_servo_manager.
 * All methods named update_...() are called before calling method terminate_now().
 */
class termination_condition
{
public:
	termination_condition();
	virtual ~termination_condition();

	/**
	 * Reset condition state.
	 * Called in first_step() of visual_servo_manager in order to reset condition state.
	 */
	virtual void reset() = 0;

	/**
	 * Implement this to indicate when generator should be terminated.
	 * @return true if generator should be terminated.
	 */
	virtual bool terminate_now() = 0;

	/**
	 * Set end effector's speed.
	 * @param current_speed
	 */
	virtual void update_end_effector_speed(const Eigen::Matrix<double, 3, 1>& current_speed);

	/**
	 * Update end effector's acceleration.
	 * @param current_accel
	 */
	virtual void update_end_effector_accel(const Eigen::Matrix<double, 3, 1>& current_accel);

	/**
	 *
	 * @param object_visible
	 */
	virtual void update_object_visibility(bool object_visible);

protected:
	Eigen::Matrix<double, 3, 1> current_speed;
	Eigen::Matrix<double, 3, 1> current_accel;
	bool object_visible;
};

/** @} */

}//namespace generator

}

}

}

#endif /* TERMINATION_CONDITION_H_ */

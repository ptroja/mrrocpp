/*
 * termination_condition.h
 *
 *  Created on: Apr 29, 2010
 *      Author: mboryn
 */

#ifndef TERMINATION_CONDITION_H_
#define TERMINATION_CONDITION_H_

#include <Eigen/Core>

#include "visual_servo_manager.h"

//class mrrocpp::ecp::common::generator::visual_servo_manager;

namespace mrrocpp {
namespace ecp {

namespace common {
namespace generator {
class visual_servo_manager;
}//namespace generator
}//namespace common

namespace servovision {

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
	 *
	 * Called in first_step() of visual_servo_manager in order to reset condition state.
	 */
	virtual void reset() = 0;

	/**
	 * Make termination condition update itself.
	 * @param vsm
	 */
	virtual void update(const mrrocpp::ecp::common::generator::visual_servo_manager* vsm) = 0;

	/**
	 * Check if condition is met.
	 * Implement this to indicate when generator should be terminated.
	 * @return true if generator should be terminated.
	 */
	bool is_condition_met() const;
protected:
	bool condition_met;
};

/** @} */

}//namespace generator
}
}

#endif /* TERMINATION_CONDITION_H_ */

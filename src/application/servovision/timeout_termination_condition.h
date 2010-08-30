/*
 * timeout_termination_condition.h
 *
 *  Created on: Jun 2, 2010
 *      Author: mboryn
 */

#ifndef TIMEOUT_TERMINATION_CONDITION_H_
#define TIMEOUT_TERMINATION_CONDITION_H_

#include "termination_condition.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class timeout_termination_condition : public termination_condition
{
public:
	timeout_termination_condition(double timeout);
	virtual ~timeout_termination_condition();
	virtual void reset();
	virtual void update(const visual_servo_manager* vsm);
	virtual bool is_condition_met() const;
protected:
	double timeout;
	double time_left;
};

/** @} */

}//namespace generator
}
}

#endif /* TIMEOUT_TERMINATION_CONDITION_H_ */

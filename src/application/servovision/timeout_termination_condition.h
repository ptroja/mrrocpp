/*
 * timeout_termination_condition.h
 *
 *  Created on: Jun 2, 2010
 *      Author: mboryn
 */

#ifndef TIMEOUT_TERMINATION_CONDITION_H_
#define TIMEOUT_TERMINATION_CONDITION_H_

#include <time.h>

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
	virtual void update(const mrrocpp::ecp::common::generator::visual_servo_manager* vsm);
protected:
	double timeout;

	double start_time;

	double get_time_s();
};

/** @} */

}//namespace generator
}
}

#endif /* TIMEOUT_TERMINATION_CONDITION_H_ */

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

namespace common {

namespace generator {

class timeout_termination_condition : public mrrocpp::ecp::common::generator::termination_condition
{
public:
	timeout_termination_condition(double timeout);
	virtual ~timeout_termination_condition();
	virtual void reset();
	virtual bool terminate_now();
protected:
	double timeout;
	double time_left;
};

}//namespace generator

}

}

}

#endif /* TIMEOUT_TERMINATION_CONDITION_H_ */

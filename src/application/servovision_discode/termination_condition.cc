/*
 * termination_condition.cc
 *
 *  Created on: Apr 29, 2010
 *      Author: mboryn
 */

#include "termination_condition.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {

termination_condition::termination_condition() : condition_met(false)
{
}

termination_condition::~termination_condition()
{
}

bool termination_condition::is_condition_met() const {
	return condition_met;
}

}//namespace servovision
}
}

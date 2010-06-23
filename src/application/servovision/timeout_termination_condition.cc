/*
 * timeout_termination_condition.cc
 *
 *  Created on: Jun 2, 2010
 *      Author: mboryn
 */

#include "timeout_termination_condition.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

timeout_termination_condition::timeout_termination_condition()
{
	// TODO Auto-generated constructor stub

}

timeout_termination_condition::~timeout_termination_condition()
{
	// TODO Auto-generated destructor stub
}

void timeout_termination_condition::reset()
{
	time_left = timeout;
}
bool timeout_termination_condition::is_condition_met()
{
	return time_left <= 0;
}

}//namespace generator

}

}

}

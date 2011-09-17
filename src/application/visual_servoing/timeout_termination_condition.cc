/*
 * timeout_termination_condition.cc
 *
 *  Created on: Jun 2, 2010
 *      Author: mboryn
 */

#include <stdexcept>

#include "timeout_termination_condition.h"
#include "base/lib/logger.h"

using namespace std;
using namespace logger;

namespace mrrocpp {
namespace ecp {
namespace servovision {

timeout_termination_condition::timeout_termination_condition(double timeout) :
	timeout(timeout)
{
}

timeout_termination_condition::~timeout_termination_condition()
{
}

void timeout_termination_condition::reset()
{
	start_time = get_time_s();

	condition_met = false;
}

void timeout_termination_condition::update(const mrrocpp::ecp::common::generator::visual_servo_manager* vsm)
{
	if (get_time_s() - start_time >= timeout) {
		condition_met = true;
	}
}

double timeout_termination_condition::get_time_s()
{
	struct timespec current_time;

	if (clock_gettime(CLOCK_REALTIME, &current_time) == -1) {
		throw std::logic_error("timeout_termination_condition::get_time_s(): clock_gettime");
	}

	return current_time.tv_sec + ((double) current_time.tv_nsec) * 1e-9;

}

}//namespace generator
}
}

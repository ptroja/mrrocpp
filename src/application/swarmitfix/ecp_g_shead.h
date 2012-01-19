/*
 * generator/ecp_g_head.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SHEAD_H_
#define ECP_G_SHEAD_H_

#include <boost/thread/thread_time.hpp>

#include "base/ecp/ecp_generator.h"
#include "robot/shead/dp_shead.h"
#include "robot/shead/ecp_r_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

class rotate : public common::generator::_generator<ecp::shead::robot>
{
public:
	rotate(task_t & _ecp_task, const double & goal); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

private:
	//! Reference to the buffer with command data
	const double & goal;

	//! Wakeup timer
	boost::system_time wakeup;

	//! Effector query interval
	const boost::posix_time::time_duration query_interval;
};

class control : public common::generator::_generator<ecp::shead::robot>
{
public:
	control(task_t & _ecp_task, const lib::shead::next_state::control_t & control); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

private:
	//! Reference to the buffer with command data
	const lib::shead::next_state::control_t & control_state;
};

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif

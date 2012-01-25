/*
 * ecp_g_spkm.h
 *
 * Author: ptroja
 */

#ifndef ECP_G_SPKM_H_
#define ECP_G_SPKM_H_

#include <boost/thread/thread_time.hpp>

#include "robot/spkm/ecp_r_spkm.h"
#include "robot/spkm/dp_spkm.h"

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace generator {

class spkm_pose : public common::generator::_generator<ecp::spkm::robot>
{
public:
	//! Constructor
	spkm_pose(task_t & _ecp_task, const lib::spkm::segment_t & _segment);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();

private:
	//! Request execution of a single motion segment
	void request_segment_execution(robot_t & robot, const lib::spkm::segment_t & segment);

	//! Motion segments
	const lib::spkm::segment_t & segment;

	//! Wakeup timer
	boost::system_time wakeup;

	//! Effector query interval
	const boost::posix_time::time_duration query_interval;
};

class spkm_quickstop : public common::generator::_generator<ecp::spkm::robot>
{
public:
	//! Constructor
	spkm_quickstop(task_t & _ecp_task);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();
};

} // namespace generator
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

#endif

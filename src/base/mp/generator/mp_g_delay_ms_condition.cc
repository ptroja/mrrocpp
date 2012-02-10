/*!
 * @file
 * @brief File contains mp delay_ms_condition generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <boost/thread/thread.hpp>
#include <boost/thread/thread_time.hpp>

#include "base/mp/mp_task.h"
#include "base/mp/generator/mp_g_delay_ms_condition.h"

namespace mrrocpp {
namespace mp {
namespace generator {

delay_ms_condition::delay_ms_condition(task::task& _mp_task, unsigned int _ms_delay) :
	generator(_mp_task)
{
	configure(_ms_delay);
}

void delay_ms_condition::configure(unsigned int _ms_delay)
{
	ms_delay = _ms_delay;
}

bool delay_ms_condition::first_step()
{
	timeout = boost::get_system_time() + boost::posix_time::milliseconds(ms_delay);

	return true;
}

bool delay_ms_condition::next_step()
{
	if (boost::get_system_time() < timeout) {
		boost::this_thread::sleep(boost::posix_time::milliseconds(20));

		return true;
	}

	return false;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


/*!
 * @file
 * @brief File contains mp delay_ms_condition generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/mp_task.h"
#include "base/mp/generator/mp_g_delay_ms_condition.h"

#include "unistd.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// condition to wait for desired time in ms

delay_ms_condition::delay_ms_condition(task::task& _mp_task, int _ms_delay) :
	generator(_mp_task)
{
	configure(_ms_delay);
}

void delay_ms_condition::configure(int _ms_delay)
{
	ms_delay = _ms_delay;
}

bool delay_ms_condition::first_step()
{
	local_timer.start();
	return true;
}

bool delay_ms_condition::next_step()
{
	float sec;
	local_timer.stop();
	local_timer.get_time(sec);
	if (1000 * sec > (float) ms_delay)
		return false;
	delay (20);
	local_timer.stop();
	local_timer.get_time(sec);
	if (1000 * sec > (float) ms_delay)
		return false;
	return true;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp


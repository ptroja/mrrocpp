/*!
 * @file
 * @brief File contains mp base task declaration
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <vector>

#include <boost/foreach.hpp>

#include "base/mp/mp_task.h"

#include "generator/mp_g_delay_ms_condition.h"
#include "generator/mp_g_wait_for_task_termination.h"

namespace mrrocpp {
namespace mp {
namespace task {

task::task(lib::configurator &_config)
	: task_base(_config)
{
}

void task::wait_ms(unsigned int _ms_delay) // zamiast delay
{
	generator::delay_ms_condition mp_ds_ms(*this, _ms_delay);

	mp_ds_ms.Move();
}

void task::wait_for_task_termination(bool activate_trigger, const std::vector <lib::robot_name_t> & robotSet)
{
	generator::wait_for_task_termination wtf_gen(*this);

	BOOST_FOREACH(lib::robot_name_t robotName, robotSet)
			{
				wtf_gen.robot_m[robotName] = robot_m[robotName];
			}

	wtf_gen.configure(activate_trigger);

	wtf_gen.Move();
}

} // namespace task
} // namespace mp
} // namespace mrrocpp

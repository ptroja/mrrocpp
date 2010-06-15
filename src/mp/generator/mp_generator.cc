#include <boost/foreach.hpp>

#include "mp/generator/mp_generator.h"
#include "mp/robot/mp_robot.h"

#include "lib/agent/OrDataCondition.h"

namespace mrrocpp {
namespace mp {
namespace generator {

generator::generator(task::task& _mp_task) :
	ecp_mp::generator::generator(*_mp_task.sr_ecp_msg), mp_t(_mp_task)
{
}

// ---------------------------------------------------------------
void generator::Move(void)
{
	OrDataCondition anyData(mp_t.ui_command_buffer);

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
		anyData = anyData | robot_node.second->ecp_reply_package_buffer;
	}

	node_counter = 0;

	// (Inicjacja) generacja pierwszego kroku ruchu
	if (!first_step())
		return;

	// realizacja ruchu
	do {
		// przygotowanie danych od czujnikow
		mp_t.all_sensors_initiate_reading(sensor_m);

		// wykonanie kroku ruchu przez wybrane roboty (z flaga 'communicate')
		mp_t.execute_all(robot_m);

		// odczytanie danych z wszystkich czujnikow
		mp_t.all_sensors_get_reading(sensor_m);

		// Wait for new data from any source
		mp_t.Wait(anyData);

		// copy fresh data from buffers
		BOOST_FOREACH(const common::robot_pair_t & robot_node, mp_t.robot_m) {
			if(robot_node.second->ecp_reply_package_buffer.isFresh()) {
				robot_node.second->ecp_reply_package = robot_node.second->ecp_reply_package_buffer.Get();
			}
		}

		node_counter++;
	} while (next_step());
}
// ------------------------------------------------------------------------

} // namespace generator
} // namespace mp
} // namespace mrrocpp


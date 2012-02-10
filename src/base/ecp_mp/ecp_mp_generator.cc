/*!
 * @file
 * @brief File contains ecp_mp base generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include <boost/foreach.hpp>

#include "base/ecp_mp/ecp_mp_generator.h"
#include "base/ecp_mp/ecp_mp_sensor_interface.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {

generator::generator(lib::sr_ecp& _sr_ecp_msg) :
	trigger(false), sr_ecp_msg(_sr_ecp_msg), node_counter(0)
{
}

generator::~generator()
{
}

bool generator::check_and_null_trigger()
{
	bool returned_value = false;
	if (trigger) {
		trigger = false;
		returned_value = true;
	}

	return returned_value;
}

void generator::set_trigger()
{
	trigger = true;
}


void generator::initiate_sensors_readings()
{
	BOOST_FOREACH(sensor_item_t & sensor_item, sensor_m)
	{
		if (sensor_item.second->base_period > 0) {
			if (sensor_item.second->current_period == sensor_item.second->base_period) {
				sensor_item.second->initiate_reading();
			}
			sensor_item.second->current_period--;
		}
	}
}

void generator::get_sensors_readings()
{
	BOOST_FOREACH(sensor_item_t & sensor_item, sensor_m)
	{
		// jesli wogole mamy robic pomiar
		if (sensor_item.second->base_period > 0) {
			if (sensor_item.second->current_period == 0) {
				sensor_item.second->get_reading();
				sensor_item.second->current_period = sensor_item.second->base_period;
			}
		}
	}
}

} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp

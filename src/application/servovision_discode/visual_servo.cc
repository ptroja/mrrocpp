/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "visual_servo.h"

using namespace logger;

namespace mrrocpp {
namespace ecp {
namespace servovision {

visual_servo::visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, boost::shared_ptr <
		mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor) :
	regulator(regulator), sensor(sensor), object_visible(false), max_steps_without_reading(5), steps_without_reading(0)
{

}

visual_servo::~visual_servo()
{
}

lib::Homog_matrix visual_servo::get_position_change(const lib::Homog_matrix& current_position, double dt)
{
	lib::Homog_matrix delta_position;

	/* TODO:
	 if (get_sensor_report() == lib::sensor::VSP_SENSOR_NOT_CONFIGURED) {
	 // Sensor not ready yet.
	 return delta_position;
	 }
	 */

	if (sensor->is_reading_ready()) {
		// There's a reading, reset the counter.
		steps_without_reading = 0;
	} else {
		// Maybe there is a valid reading
		if (steps_without_reading > max_steps_without_reading) {
			// The object is no longer visible
			object_visible = false;
			log_dbg("visual_servo::get_position_change(): object considered no longer visible\n");
			return delta_position;
		} else {
			steps_without_reading++;
		}
	}

	retrieve_reading();

	object_visible = is_object_visible_in_latest_reading();

	if (object_visible) {
		return compute_position_change(current_position, dt);
	}

	return delta_position;
} // get_position_change

bool visual_servo::is_object_visible()
{
	return object_visible;
}

const Eigen::Matrix <double, 6, 1> & visual_servo::get_error()
{
	return error;
}

boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> visual_servo::get_sensor()
{
	return sensor;
}

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp

/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_H_
#define VISUAL_SERVO_H_

#include "base/lib/mrmath/mrmath.h"
#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"
#include <boost/shared_ptr.hpp>
#include "visual_servo_regulator.h"

#include "base/ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 * Base class for visual servoing.
 * Implements two essential methods: first_step() and next_step(). Subclasses must implement method step() which is called from next_step().
 */
class visual_servo
{
public:
	virtual ~visual_servo();
	/**
	 * Calculates relative change of position.
	 * @param current_position end effector's current position.
	 * @param dt time between calls to get_position_change.
	 * @return
	 */
	lib::Homog_matrix get_position_change(const lib::Homog_matrix& current_position, double dt);

	/**
	 * Returns fradia_sensor.
	 * @return
	 */
	virtual boost::shared_ptr <ecp_mp::sensor::sensor_interface> get_vsp_fradia() = 0;

	/**
	 * Returns object visibility.
	 * @return
	 */
	bool is_object_visible();

	const Eigen::Matrix <double, 6, 1> & get_error();
protected:
	visual_servo(boost::shared_ptr <visual_servo_regulator> regulator);

	virtual lib::Homog_matrix compute_position_change(const lib::Homog_matrix& current_position, double dt) = 0;

	virtual lib::sensor::VSP_REPORT_t get_sensor_report() = 0;

	virtual bool is_object_visible_in_latest_reading() = 0;

	boost::shared_ptr <visual_servo_regulator> regulator;

	Eigen::Matrix <double, 6, 1> error;
private:
	bool object_visible;

	int max_steps_without_reading;
	int steps_without_reading;
}; // class visual_servo

/** @} */

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp

#endif /* VISUAL_SERVO_H_ */

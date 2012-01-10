/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_H_
#define VISUAL_SERVO_H_

#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "base/lib/mrmath/mrmath.h"
#include "sensor/discode/discode_sensor.h"
#include "visual_servo_regulator.h"
#include "base/ecp_mp/ecp_mp_sensor.h"
#include "Reading.h"
#include "base/lib/logger_client/logger_client.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

using logger::logger_client;
using logger::log_message;

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
	 * Returns discode_sensor.
	 * @return
	 */
	boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> get_sensor();

	virtual Types::Mrrocpp_Proxy::Reading* get_reading() = 0;

	/**
	 * Returns object visibility.
	 * @return
	 */
	bool is_object_visible();

	/**
	 * Reset visual servo state.
	 * This method should be called when visual servo motion generator is starting.
	 */
	virtual void reset();

	/**
	 *
	 * @return Error
	 */
	const Eigen::Matrix <double, 6, 1> & get_error();

	boost::shared_ptr<logger_client> log_client;
protected:
			visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, boost::shared_ptr <
					mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string& section_name, mrrocpp::lib::configurator& configurator);

	/**
	 * This method should compute position to be passed to the robot.
	 * @param current_position
	 * @param dt
	 * @return
	 */
	virtual lib::Homog_matrix compute_position_change(const lib::Homog_matrix& current_position, double dt) = 0;

	/**
	 * This method is called by get_position_change() when object is considered no longer visible.
	 * This usually happens, when object hasn't been recognized for a few macrosteps (determined by max_steps_without_reading parameter).
	 * Implemented method should reset regulator.
	 */
	virtual void notify_object_considered_not_visible();

	/**
	 * This method should retrieve reading from discode_sensor and store it for later use.
	 * This method is called only when there is new reading available.
	 */
	virtual void retrieve_reading() = 0;

	/**
	 * This method should predict reading when there was no reading available from vision sensor.
	 * This method is called when there is no fresh data available from vision sensor.
	 */
	virtual void predict_reading() = 0;

	/**
	 * This method should check latest reading, if object in that reading is visible.
	 * @return true if object was visible.
	 */
	virtual bool is_object_visible_in_latest_reading() = 0;

	/**
	 *
	 */
	boost::shared_ptr <visual_servo_regulator> regulator;
	/**
	 *
	 */
	boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor;

	Eigen::Matrix <double, 6, 1> error;
private:
	log_message msg;

	bool object_visible;

	int max_steps_without_reading;
	int steps_without_reading;

}; // class visual_servo

/** @} */

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp

#endif /* VISUAL_SERVO_H_ */

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

namespace mrrocpp {
namespace ecp {
namespace servovision {

struct visual_servo_log_sample
{
	/**
	 * Timestamp when processing in Discode starts (taken just after camera source).
	 */
	uint32_t processingStartSeconds;
	uint32_t processingStartNanoseconds;

	/**
	 * Timestamp when processing in Discode ends (taken just before sending to mrroc proxy).
	 */
	uint32_t processingEndSeconds;
	uint32_t processingEndNanoseconds;

	/** Time, when request for reading was sent from mrrocpp to discode. */
	uint32_t requestSentTimeSeconds;
	uint32_t requestSentTimeNanoseconds;

	/** Time, when reading was sent to mrrocpp. */
	uint32_t sendTimeSeconds;
	uint32_t sendTimeNanoseconds;

	/** Time, when reading was received in mrrocpp. */
	uint32_t receiveTimeSeconds;
	uint32_t receiveTimeNanoseconds;

	/** Time, when sample was taken. */
	uint32_t sampleTimeSeconds;
	uint32_t sampleTimeNanoseconds;

	double mrroc_discode_time_offset;

	/** Is object visible in latest reading. */
	bool is_object_visible;

	bool is_reading_repreated;

	static void printHeader(std::ostream& os);
	void print(std::ostream& os, uint64_t t0);
};

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
	 *
	 * @return Error
	 */
	const Eigen::Matrix <double, 6, 1> & get_error();
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
	bool object_visible;

	int max_steps_without_reading;
	int steps_without_reading;

	boost::circular_buffer <visual_servo_log_sample> log_buffer;
	static const int log_buffer_default_capacity = 500;

	void write_log();

	bool log_enabled;
}; // class visual_servo

/** @} */

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp

#endif /* VISUAL_SERVO_H_ */

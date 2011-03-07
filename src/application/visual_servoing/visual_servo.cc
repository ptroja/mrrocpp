/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include <ctime>
#include <iostream>
#include <fstream>

#include "visual_servo.h"

#include "base/lib/logger.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

using namespace logger;
using mrrocpp::ecp_mp::sensor::discode::discode_sensor;
using namespace std;

visual_servo::visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, boost::shared_ptr <
		mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor) :
	regulator(regulator), sensor(sensor), object_visible(false), max_steps_without_reading(5),
			steps_without_reading(0), time_log_filename("visual_servo_time_log.csv"), log_buffer(2000)
{
	//log_dbg("visual_servo::visual_servo() begin\n");
}

visual_servo::~visual_servo()
{
}

lib::Homog_matrix visual_servo::get_position_change(const lib::Homog_matrix& current_position, double dt)
{
	//	log_dbg("visual_servo::get_position_change(): begin\n");
	visual_servo_log_sample sample;
	memset(&sample, 0, sizeof(visual_servo_log_sample));

	struct timespec ts;
	if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
		sample.sampleTimeNanoseconds = ts.tv_nsec;
		sample.sampleTimeSeconds = ts.tv_sec;
	}

	lib::Homog_matrix delta_position;

	if (sensor->get_state() == discode_sensor::DSS_READING_RECEIVED) {
		// There's a reading, reset the counter.
		steps_without_reading = 0;

		mrrocpp::ecp_mp::sensor::discode::reading_message_header rmh = sensor->get_rmh();
		sample.readingTimeNanoseconds = rmh.readingTimeNanoseconds;
		sample.readingTimeSeconds = rmh.readingTimeSeconds;
		sample.sendTimeNanoseconds = rmh.sendTimeNanoseconds;
		sample.sendTimeSeconds = rmh.sendTimeSeconds;

		struct timespec ts = sensor->get_receive_time();

		sample.receivedTimeNanoseconds = ts.tv_nsec;
		sample.receivedTimeSeconds = ts.tv_sec;

		retrieve_reading();
	} else {
		// Maybe there is a valid reading
		steps_without_reading++;
	}

	if (steps_without_reading > max_steps_without_reading) {
		// The object is no longer visible
		object_visible = false;
	} else {
		object_visible = is_object_visible_in_latest_reading();
	}

	sample.is_object_visible = object_visible;

	if (object_visible) {
		//		log_dbg("visual_servo::get_position_change(): object_visible\n");
		delta_position = compute_position_change(current_position, dt);
	}

	log_buffer.push_back(sample);

	if (log_buffer.full()) {
		write_log();
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

void visual_servo::write_log()
{
	ofstream os;
	os.open(time_log_filename.c_str(), ofstream::out | ofstream::trunc);
	os
			<< "sampleTimeSeconds;sampleTimeNanoseconds;readingTimeSeconds;readingTimeNanoseconds;sendTimeSeconds;sendTimeNanoseconds;receivedTimeSeconds;receivedTimeNanoseconds;is_object_visible\n";
	boost::circular_buffer <visual_servo_log_sample>::iterator it;
	for (it = log_buffer.begin(); it != log_buffer.end(); ++it) {
		os << it->sampleTimeSeconds << ";";
		os << it->sampleTimeNanoseconds << ";";
		os << it->readingTimeSeconds << ";";
		os << it->readingTimeNanoseconds << ";";
		os << it->sendTimeSeconds << ";";
		os << it->sendTimeNanoseconds << ";";
		os << it->receivedTimeSeconds << ";";
		os << it->receivedTimeNanoseconds << ";";
		os << it->is_object_visible;
		os << "\n";
	}
	os.close();
	log_buffer.clear();
}

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp

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
		mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	regulator(regulator), sensor(sensor), object_visible(false), max_steps_without_reading(5),
			steps_without_reading(0), log_enabled(false)
{
	//log_dbg("visual_servo::visual_servo() begin\n");

	string log_enabled_name = "vs_log_enabled";
	string log_buffer_size_name = "vs_log_buffer_size";

	if (configurator.exists(log_enabled_name, section_name)
			&& configurator.value <bool> (log_enabled_name, section_name)) {
		log_enabled = true;
		if (configurator.exists(log_buffer_size_name, section_name)) {
			log_buffer.set_capacity(configurator.value <unsigned int> (log_buffer_size_name, section_name));
		} else {
			log_buffer.set_capacity(log_buffer_default_capacity);
		}
	}
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
		sample.imageSourceTimeNanoseconds = rmh.imageSourceTimeNanoseconds;
		sample.imageSourceTimeSeconds = rmh.imageSourceTimeSeconds;
		sample.sendTimeNanoseconds = rmh.sendTimeNanoseconds;
		sample.sendTimeSeconds = rmh.sendTimeSeconds;

		struct timespec ts = sensor->get_reading_received_time();
		sample.receiveTimeNanoseconds = ts.tv_nsec;
		sample.receiveTimeSeconds = ts.tv_sec;

		ts = sensor->get_request_sent_time();
		sample.requestSentTimeNanoseconds = ts.tv_nsec;
		sample.requestSentTimeSeconds = ts.tv_sec;

		retrieve_reading();
	} else {
		// Maybe there is a valid reading
		steps_without_reading++;
	}

	if (steps_without_reading > max_steps_without_reading) {
		// The object is no longer visible
		log_dbg("visual_servo::get_position_change(): steps_without_reading > max_steps_without_reading.\n");
		object_visible = false;
	} else {
		object_visible = is_object_visible_in_latest_reading();
	}

	sample.is_object_visible = object_visible;

	if (object_visible) {
		//		log_dbg("visual_servo::get_position_change(): object_visible, calling compute_position_change\n");
		delta_position = compute_position_change(current_position, dt);
	} else {
		notify_object_considered_not_visible();
	}

	if (log_enabled) {
		log_buffer.push_back(sample);
		if (log_buffer.size() % 100 == 0) {
			log_dbg("log_buffer.size(): %d\n", (int) log_buffer.size());
		}
		if (log_buffer.full()) {
			write_log();
		}
	}

	//	log_dbg("visual_servo::get_position_change(): end\n");
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

void visual_servo::notify_object_considered_not_visible()
{
	regulator->reset();
}

void visual_servo::write_log()
{
	log("visual_servo::write_log() begin\n");

	time_t timep = time(NULL);
	struct tm* time_split = localtime(&timep);
	char time_log_filename[128];
	sprintf(time_log_filename, "../../msr/%04d-%02d-%02d_%02d-%02d-%02d_VS.csv", time_split->tm_year + 1900, time_split->tm_mon
			+ 1, time_split->tm_mday, time_split->tm_hour, time_split->tm_min, time_split->tm_sec);

	ofstream os;
	os.open(time_log_filename, ofstream::out | ofstream::trunc);

	visual_servo_log_sample::printHeader(os);

	boost::circular_buffer <visual_servo_log_sample>::iterator it;
	uint64_t t0 = 0;
	for (it = log_buffer.begin(); it != log_buffer.end(); ++it) {
		if (it == log_buffer.begin() || t0 == 0) {
			t0 = it->imageSourceTimeSeconds + it->imageSourceTimeNanoseconds * 1e-9;
		}
		it->print(os, t0);
	}
	os.close();
	log_buffer.clear();
	log("visual_servo::write_log() end\n");
}

void visual_servo_log_sample::print(std::ostream& os, uint64_t t0)
{
	double sampleTime = (sampleTimeSeconds - t0) + sampleTimeNanoseconds * 1e-9;
	double imageSourceTime = (imageSourceTimeSeconds - t0) + imageSourceTimeNanoseconds * 1e-9;
	double sendTime = (sendTimeSeconds - t0) + sendTimeNanoseconds * 1e-9;
	double requestSentTime = (requestSentTimeSeconds - t0) + requestSentTimeNanoseconds * 1e-9;
	double receiveTime = (receiveTimeSeconds - t0) + receiveTimeNanoseconds * 1e-9;

	if (imageSourceTimeSeconds > 0) {
		os.precision(9);
		os << fixed << imageSourceTime << ";";
		os << imageSourceTimeSeconds << ";";
		os << imageSourceTimeNanoseconds << ";";
	} else {
		os << ";";
		os << ";";
		os << ";";
	}

	if (sendTimeSeconds > 0) {
		os.precision(9);
		os << fixed << sendTime << ";";
		os << sendTimeSeconds << ";";
		os << sendTimeNanoseconds << ";";
	} else {
		os << ";";
		os << ";";
		os << ";";
	}

	if (requestSentTimeSeconds > 0) {
		os.precision(9);
		os << fixed << requestSentTime << ";";
		os << requestSentTimeSeconds << ";";
		os << requestSentTimeNanoseconds << ";";
	} else {
		os << ";";
		os << ";";
		os << ";";
	}

	if (receiveTimeSeconds > 0) {
		os.precision(9);
		os << fixed << receiveTime << ";";
		os << receiveTimeSeconds << ";";
		os << receiveTimeNanoseconds << ";";
	} else {
		os << ";";
		os << ";";
		os << ";";
	}

	if (sampleTimeSeconds > 0) {
		os.precision(9);
		os << fixed << sampleTime << ";";
		os << sampleTimeSeconds << ";";
		os << sampleTimeNanoseconds << ";";
	} else {
		os << ";";
		os << ";";
		os << ";";
	}

	os << is_object_visible << ";";

	if (receiveTimeSeconds > 0 && sendTimeSeconds > 0) {
		os.precision(9);
		os << fixed << (receiveTime - sendTime) << ";";
	} else {
		os << ";";
	}

	if (sampleTimeSeconds > 0 && imageSourceTimeSeconds > 0) {
		os.precision(9);
		os << fixed << (sampleTime - imageSourceTime) << ";";
	} else {
		os << ";";
	}

	os << "\n";
}

void visual_servo_log_sample::printHeader(std::ostream& os)
{
	os << "imageSourceTime;";
	os << "imageSourceTime [s];";
	os << "imageSourceTime [ns];";

	os << "sendTime;";
	os << "sendTime [s];";
	os << "sendTime [ns];";

	os << "requestSentTime;";
	os << "requestSentTime [s];";
	os << "requestSentTime [ns];";

	os << "receiveTime;";
	os << "receiveTime [s];";
	os << "receiveTime [ns];";

	os << "sampleTime;";
	os << "sampleTime [s];";
	os << "sampleTime [ns];";

	os << "is_object_visible;";

	os << "communication delay [s];";
	os << "transport delay [s];";

	os << "\n";
}

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp

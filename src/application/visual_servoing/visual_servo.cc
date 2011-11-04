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
	regulator(regulator), sensor(sensor), object_visible(false), max_steps_without_reading(5), steps_without_reading(0)
{
	log_dbg("visual_servo::visual_servo() begin\n");

	string log_enabled_name = "vs_log_enabled";
	if (configurator.exists(log_enabled_name, section_name)
			&& configurator.value <bool> (log_enabled_name, section_name)) {
		unsigned int capacity = configurator.value <unsigned int> ("vs_log_capacity", section_name);
		std::string server_addr = configurator.value <std::string> ("vs_log_server_addr", section_name);
		int server_port = configurator.value <int> ("vs_log_server_port", section_name);

		log_client = boost::shared_ptr <logger_client>(new logger_client(capacity, server_addr, server_port));
	}

	log_dbg("visual_servo::visual_servo() end\n");
}

visual_servo::~visual_servo()
{
}

lib::Homog_matrix visual_servo::get_position_change(const lib::Homog_matrix& current_position, double dt)
{
	//	log_dbg("visual_servo::get_position_change(): begin\n");

	lib::Homog_matrix delta_position;

	struct timespec sendTime, receiveTime, requestSentTime, processingStart, processingEnd;
	sendTime.tv_sec = sendTime.tv_nsec = 0;
	receiveTime.tv_sec = receiveTime.tv_nsec = 0;
	requestSentTime.tv_sec = requestSentTime.tv_nsec = 0;
	processingStart.tv_sec = processingStart.tv_nsec = 0;
	processingEnd.tv_sec = processingEnd.tv_nsec = 0;

	double mrroc_discode_time_offset = 0;
	bool is_reading_repreated = false;

	switch (sensor->get_state())
	{
		case discode_sensor::DSS_READING_RECEIVED: {
			// There's a reading, reset the counter.
			steps_without_reading = 0;

			mrrocpp::ecp_mp::sensor::discode::reading_message_header rmh = sensor->get_rmh();
			sendTime.tv_nsec = rmh.sendTimeNanoseconds;
			sendTime.tv_sec = rmh.sendTimeSeconds;

			receiveTime = sensor->get_reading_received_time();

			requestSentTime = sensor->get_request_sent_time();

			mrroc_discode_time_offset = sensor->get_mrroc_discode_time_offset();

			retrieve_reading();
			Types::Mrrocpp_Proxy::Reading* reading = get_reading();

			processingStart.tv_sec = reading->processingStartSeconds;
			processingStart.tv_nsec = reading->processingStartNanoseconds;

			processingEnd.tv_sec = reading->processingEndSeconds;
			processingEnd.tv_nsec = reading->processingEndNanoseconds;
		}
			break;
		case discode_sensor::DSS_CONNECTED: // processing in DisCODe hasn't finished yet
		case discode_sensor::DSS_REQUEST_SENT: // communication or synchronisation in DisCODe took too long
			predict_reading();
			steps_without_reading++;
			is_reading_repreated = true;
			break;
		default: // error
			log_dbg("visual_servo::get_position_change(): error\n");
	}

	if (steps_without_reading > max_steps_without_reading) {
		// The object is no longer visible
		log_dbg("visual_servo::get_position_change(): steps_without_reading > max_steps_without_reading.\n");
		object_visible = false;
	} else {
		object_visible = is_object_visible_in_latest_reading();
	}

	if (object_visible) {
		//		log_dbg("visual_servo::get_position_change(): object_visible, calling compute_position_change\n");
		delta_position = compute_position_change(current_position, dt);
	} else {
		notify_object_considered_not_visible();
	}

	msg.time_elems = 5;
	msg.time_buf[0] = requestSentTime;
	msg.time_buf[1] = sendTime;
	msg.time_buf[2] = receiveTime;
	msg.time_buf[3] = processingStart;
	msg.time_buf[4] = processingEnd;

	// TODO: prepare log message
	if (sendTime.tv_sec > 0) {
		sprintf(msg.text, "%d;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%.6lf", (int) object_visible, requestSentTime.tv_sec, requestSentTime.tv_nsec, sendTime.tv_sec, sendTime.tv_nsec, receiveTime.tv_sec, receiveTime.tv_nsec, processingStart.tv_sec, processingStart.tv_nsec, processingEnd.tv_sec, processingEnd.tv_nsec, mrroc_discode_time_offset);
	} else {
		sprintf(msg.text, "%d", (int) object_visible);
	}

	// write log message
	if (log_client.get() != NULL) {
		log_client->log(msg);
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

//void visual_servo::write_log()
//{
//	log("visual_servo::write_log() begin\n");
//
//	time_t timep = time(NULL);
//	struct tm* time_split = localtime(&timep);
//	char time_log_filename[128];
//	sprintf(time_log_filename, "../../msr/%04d-%02d-%02d_%02d-%02d-%02d_VS.csv", time_split->tm_year + 1900, time_split->tm_mon
//			+ 1, time_split->tm_mday, time_split->tm_hour, time_split->tm_min, time_split->tm_sec);
//
//	ofstream os;
//	os.open(time_log_filename, ofstream::out | ofstream::trunc);
//
//	visual_servo_log_sample::printHeader(os);
//
//	boost::circular_buffer <visual_servo_log_sample>::iterator it;
//	uint64_t t0 = 0;
//	for (it = log_buffer.begin(); it != log_buffer.end(); ++it) {
//		if (it == log_buffer.begin() || t0 == 0) {
//			t0 = it->processingStartSeconds + it->processingStartNanoseconds * 1e-9;
//		}
//		it->print(os, t0);
//	}
//	os.close();
//	log_buffer.clear();
//	log("visual_servo::write_log() end\n");
//}
//
//void visual_servo_log_sample::print(std::ostream& os, uint64_t t0)
//{
//	double sampleTime = (sampleTimeSeconds - t0) + sampleTimeNanoseconds * 1e-9;
//	double processingStart = (processingStartSeconds - t0) + processingStartNanoseconds * 1e-9
//			+ mrroc_discode_time_offset;
//	double processingEnd = (processingEndSeconds - t0) + processingEndNanoseconds * 1e-9 + mrroc_discode_time_offset;
//	double sendTime = (sendTimeSeconds - t0) + sendTimeNanoseconds * 1e-9 + mrroc_discode_time_offset;
//	double requestSentTime = (requestSentTimeSeconds - t0) + requestSentTimeNanoseconds * 1e-9;
//	double receiveTime = (receiveTimeSeconds - t0) + receiveTimeNanoseconds * 1e-9;
//
//	if (!is_reading_repreated) {
//		os << mrroc_discode_time_offset;
//	}
//	os << ";";
//
//	if (processingStartSeconds > 0) {
//		os.precision(9);
//		os << fixed << processingStart << ";";
//	} else {
//		os << ";";
//	}
//	if (processingEndSeconds > 0) {
//		os.precision(9);
//		os << fixed << processingEnd << ";";
//	} else {
//		os << ";";
//	}
//
//	if (sendTimeSeconds > 0) {
//		os.precision(9);
//		os << fixed << sendTime << ";";
//	} else {
//		os << ";";
//	}
//
//	if (requestSentTimeSeconds > 0) {
//		os.precision(9);
//		os << fixed << requestSentTime << ";";
//	} else {
//		os << ";";
//	}
//
//	if (receiveTimeSeconds > 0) {
//		os.precision(9);
//		os << fixed << receiveTime << ";";
//	} else {
//		os << ";";
//	}
//
//	if (sampleTimeSeconds > 0) {
//		os.precision(9);
//		os << fixed << sampleTime << ";";
//	} else {
//		os << ";";
//	}
//
//	os << is_object_visible << ";";
//
//	os << is_reading_repreated << ";";
//
//	os << "\n";
//}
//
//void visual_servo_log_sample::printHeader(std::ostream& os)
//{
//	os << "mrroc_discode_time_offset;";
//
//	os << "processingStart;";
//
//	os << "processingEnd;";
//
//	os << "sendTime;";
//
//	os << "requestSentTime;";
//
//	os << "receiveTime;";
//
//	os << "sampleTime;";
//
//	os << "is_object_visible;";
//
//	os << "is_reading_repreated;";
//
//	os << "\n";
//}

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp

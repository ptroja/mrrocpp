/*
 * single_visual_servo_manager.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include <stdexcept>

#include "single_visual_servo_manager.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

using namespace logger;
using namespace std;

single_visual_servo_manager::single_visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> vs) :
	visual_servo_manager(ecp_task, section_name), image_sampling_period(0)
{
	servos.push_back(vs);

	macrostep_length_control
			= ecp_task.config.exists("macrostep_length_control", section_name) ? ecp_task.config.value <bool> ("macrostep_length_control", section_name) : false;
	if (macrostep_length_control) {
		image_sampling_period = ecp_task.config.value <double> ("image_sampling_period", section_name);
		sr_ecp_msg.message("Using macrostep length control");
	} else {
		sr_ecp_msg.message("Macrostep length control disabled");
	}
}

single_visual_servo_manager::~single_visual_servo_manager()
{
}

lib::Homog_matrix single_visual_servo_manager::get_aggregated_position_change()
{
	bool reading_received = false;
	ecp_mp::sensor::discode::reading_message_header rmh;
	if (macrostep_length_control && servos[0]->get_sensor()->get_state()
			== ecp_mp::sensor::discode::discode_sensor::DSS_READING_RECEIVED) {
		rmh = servos[0]->get_sensor()->get_rmh();
		reading_received = true;
	}

	lib::Homog_matrix pc = servos[0]->get_position_change(get_current_position(), get_dt());

	if (macrostep_length_control && reading_received) {
		update_motion_steps(rmh);
	} else if (macrostep_length_control) {
		set_new_motion_steps(get_motion_steps_base());
	}

	return pc;
}

void single_visual_servo_manager::configure_all_servos()
{
}

void single_visual_servo_manager::update_motion_steps(ecp_mp::sensor::discode::reading_message_header rmh)
{
	Types::Mrrocpp_Proxy::Reading* reading = servos[0]->get_reading();

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);

	int seconds = ts.tv_sec - reading->processingStartSeconds;
	int nanoseconds = ts.tv_nsec - reading->processingStartNanoseconds;
	double image_mrroc_delay = seconds + 1e-9 * nanoseconds;
	image_mrroc_delay -= servos[0]->get_sensor()->get_mrroc_discode_time_offset();

	double offset = fmod(image_mrroc_delay, image_sampling_period);

	if (offset > image_sampling_period / 2) {
		offset -= image_sampling_period;
	}

	double offset_threshold = image_sampling_period / 20;

	int ms;

	if (offset > offset_threshold) {
		ms = get_motion_steps_base() - 1;
	} else if (offset < -offset_threshold) {
		ms = get_motion_steps_base() + 1;
	} else {
		ms = get_motion_steps_base();
	}

	set_new_motion_steps(ms);
}

}//namespace generator

}

}

}

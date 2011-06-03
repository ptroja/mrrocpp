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

const double single_visual_servo_manager::image_sampling_period_default = 0.040;

single_visual_servo_manager::single_visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> vs) :
	visual_servo_manager(ecp_task, section_name)
{
	servos.push_back(vs);

	image_sampling_period = ecp_task.config.exists("image_sampling_period", section_name) ? ecp_task.config.value <
			double> ("image_sampling_period", section_name) : image_sampling_period_default;
	txtiter=0;
}

single_visual_servo_manager::~single_visual_servo_manager()
{
}

lib::Homog_matrix single_visual_servo_manager::get_aggregated_position_change()
{
	//	log_dbg("single_visual_servo_manager::get_aggregated_position_change() begin\n");
	ecp_mp::sensor::discode::reading_message_header rmh;
	bool new_reading_received = false;
	Types::Mrrocpp_Proxy::Reading* reading;

	if (servos[0]->get_sensor()->get_state() == ecp_mp::sensor::discode::discode_sensor::DSS_READING_RECEIVED) {
		rmh = servos[0]->get_sensor()->get_rmh();
		reading = servos[0]->get_reading();
		new_reading_received = true;
	}

	lib::Homog_matrix pc = servos[0]->get_position_change(get_current_position(), get_dt());

	if (new_reading_received) {
		double discode_processing_time;
		double discode_synchronization_delay;
		double discode_total_time;
		double image_mrroc_delay;
		char txt[1024];


		sprintf(txt, "rmh.sendTimeSeconds = %d    rmh.sendTimeNanoseconds = %d    \n", rmh.sendTimeSeconds, rmh.sendTimeNanoseconds);
		txtbuf += txt;

		sprintf(txt, "reading->processingStartSeconds = %d    reading->processingStartNanoseconds = %d    \n", reading->processingStartSeconds, reading->processingStartNanoseconds);
		txtbuf += txt;

		sprintf(txt, "reading->processingEndSeconds = %d    reading->processingEndNanoseconds = %d    \n", reading->processingEndSeconds, reading->processingEndNanoseconds);
		txtbuf += txt;

		int seconds, nanoseconds;

		seconds = rmh.sendTimeSeconds - reading->processingEndSeconds;
		nanoseconds = rmh.sendTimeNanoseconds - reading->processingEndNanoseconds;
		discode_synchronization_delay = seconds + nanoseconds * 1e-9;

		seconds = reading->processingEndSeconds - reading->processingStartSeconds;
		nanoseconds = reading->processingEndNanoseconds - reading->processingStartNanoseconds;
		discode_processing_time = seconds + nanoseconds * 1e-9;

		seconds = rmh.sendTimeSeconds - reading->processingStartSeconds;
		nanoseconds = rmh.sendTimeNanoseconds - reading->processingStartNanoseconds;
		discode_total_time = seconds + nanoseconds * 1e-9;

		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		sprintf(txt, "ts.tv_sec = %d    ts.tv_nsec = %d    \n", ts.tv_sec, ts.tv_nsec);
		txtbuf += txt;

		seconds = ts.tv_sec - reading->processingStartSeconds;
		nanoseconds = ts.tv_nsec - reading->processingStartNanoseconds;
		image_mrroc_delay = seconds + 1e-9*nanoseconds;

		update_motion_steps(discode_processing_time, discode_synchronization_delay, discode_total_time, image_mrroc_delay);

		txtiter++;
		if(txtiter > 100){
			log("\n\n\nHEHEHEHE:\n%s\n\n", txtbuf.c_str());
			throw runtime_error("HEHEHEEHEH");
		}
	}

	//	log_dbg("single_visual_servo_manager::get_aggregated_position_change() end\n");

	return pc;
}

void single_visual_servo_manager::configure_all_servos()
{
}

void single_visual_servo_manager::update_motion_steps(double discode_processing_time, double discode_synchronization_delay, double discode_total_time, double image_mrroc_delay)
{

	//
	//log_dbg("t_p = %g    t_w = %g    t_p + t_w = %g\n", discode_processing_time, discode_synchronization_delay, discode_total_time);
	//
	//	if (discode_synchronization_delay > image_sampling_period) {
	//		//set_new_motion_steps(get_motion_steps() - image_sampling_period / step_time);
	//		set_new_motion_steps(get_motion_steps() - 1);
	//	} else {
	//		if (discode_total_time > n * image_sampling_period) {
	//			set_new_motion_steps(get_motion_steps() - 1);
	//		}
	//		if (discode_total_time < n * image_sampling_period) {
	//			set_new_motion_steps(get_motion_steps() + 1);
	//		}
	//	}

	//	int n = discode_processing_time / image_sampling_period + 1;
	//
	//	double desired_macrostep_time = image_sampling_period * n;
	//	double current_macrostep_time = get_motion_steps() * step_time;
	//	if (fabs(desired_macrostep_time - current_macrostep_time) > image_sampling_period) {
	//		set_new_motion_steps(desired_macrostep_time / step_time);
	//	} else {
	//		if (discode_total_time > n * image_sampling_period) {
	//			set_new_motion_steps(get_motion_steps() - 1);
	//		}
	//		if (discode_total_time < n * image_sampling_period) {
	//			set_new_motion_steps(get_motion_steps() + 1);
	//		}
	//	}


	//	int n = discode_processing_time / image_sampling_period + 1;
	//
	//	double desired_macrostep_time = image_sampling_period * n;
	//	double current_macrostep_time = get_motion_steps() * step_time;
	//	if (fabs(desired_macrostep_time - current_macrostep_time) > 2*image_sampling_period) {
	//		set_new_motion_steps(desired_macrostep_time / step_time);
	//	} else {
	double e = fmod(image_mrroc_delay, image_sampling_period);

//	if (e > image_sampling_period / 2) {
//		e -= image_sampling_period;
//	}

	char txt[30];
	sprintf(txt, "e = %g\n", e);
	txtbuf += txt;

	//		double eee = image_sampling_period/20;
	//		if(e > eee){
	//			set_new_motion_steps(get_motion_steps() + 1);
	//		} else if (e < -eee){
	//
	//		}
	//		if (e > image_sampling_period / 2) {
	//			e -= image_sampling_period;
	//		}
	//
	//		int new_step = get_motion_steps() + 0.1 * (e / step_time);
	//
	//		log_dbg("e = %g     new_step = %d\n", e, new_step);
	//
	//		set_new_motion_steps(new_step);
	//	}
}

}//namespace generator

}

}

}

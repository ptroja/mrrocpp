/*
 * single_visual_servo_manager.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include "single_visual_servo_manager.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

using namespace logger;

single_visual_servo_manager::single_visual_servo_manager(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> vs) :
	visual_servo_manager(ecp_task, section_name)
{
	servos.push_back(vs);
}

single_visual_servo_manager::~single_visual_servo_manager()
{
}

lib::Homog_matrix single_visual_servo_manager::get_aggregated_position_change()
{
	log_dbg("single_visual_servo_manager::get_aggregated_position_change() begin\n");
	ecp_mp::sensor::discode::reading_message_header rmh;
	bool new_reading_received = false;
	Types::Mrrocpp_Proxy::Reading* reading;

	if(servos[0]->get_sensor()->get_state() == ecp_mp::sensor::discode::discode_sensor::DSS_READING_RECEIVED){
		rmh = servos[0]->get_sensor()->get_rmh();
		reading = servos[0]->get_reading();
		new_reading_received = true;
	}

	lib::Homog_matrix pc = servos[0]->get_position_change(get_current_position(), get_dt());

	if(new_reading_received){
		double discode_processing_time;
		double discode_synchronization_delay;
		double discode_total_time;
		discode_processing_time = rmh.sendTimeSeconds - reading->processingEndSeconds + 1e-9 * (rmh.sendTimeNanoseconds - reading->processingEndNanoseconds);
		discode_synchronization_delay = reading->processingEndSeconds - reading->processingStartSeconds + 1e-9 * (reading->processingEndNanoseconds - reading->processingStartNanoseconds);
		discode_total_time = rmh.sendTimeSeconds - reading->processingStartSeconds + 1e-9 * (rmh.sendTimeNanoseconds - reading->processingStartNanoseconds);
		update_motion_steps(discode_processing_time, discode_synchronization_delay, discode_total_time);
	}

	return pc;
}

void single_visual_servo_manager::configure_all_servos()
{
}

void single_visual_servo_manager::update_motion_steps(double discode_processing_time, double discode_synchronization_delay, double discode_total_time)
{
	log_dbg("discode_processing_time = %g     discode_synchronization_delay = %g            discode_total_time = %g\n", discode_processing_time, discode_synchronization_delay, discode_total_time);

	double discode_processing_time_min = 0.02, discode_processing_time_max = 0.035;
	double discode_synchronization_delay_min = 0.005, discode_synchronization_delay_max = 0.008;

	if(discode_processing_time_min <= discode_processing_time && discode_processing_time <= discode_processing_time_max){
		if(discode_synchronization_delay > discode_synchronization_delay_max){
			set_new_motion_steps(get_motion_steps() - 1);
		}
		if(discode_synchronization_delay < discode_synchronization_delay_min){
			set_new_motion_steps(get_motion_steps() + 1);
		}
	}
}

}//namespace generator

}

}

}

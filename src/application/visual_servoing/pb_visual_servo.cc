/*
 * pb_visual_servo.cc
 *
 *  Created on: May 26, 2010
 *      Author: mboryn
 */

#include <stdexcept>
#include <ctime>

#include "pb_visual_servo.h"
#include "base/lib/logger.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

using namespace logger;
using namespace std;
using namespace mrrocpp::ecp_mp::sensor::discode;

pb_visual_servo::pb_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, boost::shared_ptr <
		mrrocpp::ecp_mp::sensor::discode::discode_sensor> sensor, const std::string& section_name, mrrocpp::lib::configurator& configurator) :
	visual_servo(regulator, sensor, section_name, configurator)
{
	reading.objectVisible = false;
	//log_dbg("pb_visual_servo::pb_visual_servo() begin\n");
	lib::Homog_matrix E_T_G_desired = configurator.value <3, 4> ("E_T_G_desired", section_name);
	G_T_E_desired = !E_T_G_desired;

	use_reading_linear_extrapolation
			= configurator.exists("use_reading_linear_extrapolation", section_name) ? configurator.value <bool> ("use_reading_linear_extrapolation", section_name) : false;

	//log_dbg("pb_visual_servo::pb_visual_servo() end\n");
}

pb_visual_servo::~pb_visual_servo()
{
}

void pb_visual_servo::retrieve_reading()
{
	try {
		//		log_dbg("pb_visual_servo::retrieve_reading()\n");
		if (sensor->get_state() == discode_sensor::DSS_READING_RECEIVED) {
			//			log_dbg("pb_visual_servo::retrieve_reading(): sensor->get_state() == discode_sensor::DSS_READING_RECEIVED.\n");

			reading_t_minus_2 = reading_t_minus_1;
			reading_t_minus_1 = reading = sensor->retreive_reading <Types::Mrrocpp_Proxy::PBReading> ();
		}
	} catch (exception &ex) {
		log("pb_visual_servo::retrieve_reading(): %s\n", ex.what());
	}
}

void pb_visual_servo::predict_reading()
{
	if (!use_reading_linear_extrapolation) {
		return;
	}

	if(reading_t_minus_2.processingStartSeconds == 0){
		return;
	}

	if( !( reading_t_minus_2.objectVisible && reading_t_minus_1.objectVisible )){
		return;
	}

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);

//	log_dbg("pb_visual_servo::predict_reading(): predicting reading\n");
	lib::Homog_matrix hm0 = reading_t_minus_2.objectPosition;
	lib::Homog_matrix hm1 = reading_t_minus_1.objectPosition;

	int sec = reading_t_minus_1.processingStartSeconds - reading_t_minus_2.processingStartSeconds;
	int nsec = reading_t_minus_1.processingStartNanoseconds - reading_t_minus_2.processingStartNanoseconds;
	double delta_t_1 = sec + 1e-9*nsec;

	sec = ts.tv_sec - reading_t_minus_2.processingStartSeconds;
	nsec = ts.tv_nsec - reading_t_minus_2.processingStartNanoseconds;
	double delta_t_2 = sec + 1e-9*nsec;

	double t = delta_t_2 / delta_t_1;

	lib::Homog_matrix hm_p = hm0.interpolate(t, hm1);

	reading.objectVisible = true;
	reading.objectPosition = hm_p;

//	log_dbg("pb_visual_servo::predict_reading() end");
}

Types::Mrrocpp_Proxy::PBReading* pb_visual_servo::get_reading()
{
	return &reading;
}

void pb_visual_servo::reset()
{
	visual_servo::reset();
	reading.objectVisible = reading_t_minus_2.objectVisible = reading_t_minus_1.objectVisible = false;
}

bool pb_visual_servo::is_object_visible_in_latest_reading()
{
	//	log_dbg("pb_visual_servo::is_object_visible_in_latest_reading(): reading.objectVisible = %d\n", (int)reading.objectVisible);
	return reading.objectVisible;
}

}//namespace generator {

}

}

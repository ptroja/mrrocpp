/*
 * pb_visual_servo.h
 *
 *  Created on: May 26, 2010
 *      Author: mboryn
 */

#ifndef PB_VISUAL_SERVO_H_
#define PB_VISUAL_SERVO_H_

#include "visual_servo.h"
#include "visual_servo_types.h"

using visual_servo_types::position_based_configuration;
using visual_servo_types::position_based_reading;

namespace mrrocpp {

namespace ecp {

namespace servovision {

typedef ecp_mp::sensor::fradia_sensor <position_based_configuration, position_based_reading> pb_fradia_sensor;

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class pb_visual_servo : public visual_servo
{
public:
	pb_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator, const std::string& section_name, mrrocpp::lib::configurator& configurator);
	virtual ~pb_visual_servo();
	virtual boost::shared_ptr <ecp_mp::sensor::sensor_interface> get_vsp_fradia();
protected:
	virtual lib::sensor::VSP_REPORT_t get_sensor_report();
	virtual bool is_object_visible_in_latest_reading();

	boost::shared_ptr <pb_fradia_sensor> vsp_fradia;

	lib::Homog_matrix G_T_E_desired;
};

/** @} */

}//namespace generator {

}

}

#endif /* PB_VISUAL_SERVO_H_ */

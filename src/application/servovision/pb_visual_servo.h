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

namespace common {

namespace generator {

typedef ecp_mp::sensor::fradia_sensor <position_based_configuration, position_based_reading> pb_fradia_sensor;

class pb_visual_servo : public mrrocpp::ecp::common::generator::visual_servo
{
public:
	pb_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator);
	virtual ~pb_visual_servo();
};

}//namespace generator {

}

}

}

#endif /* PB_VISUAL_SERVO_H_ */

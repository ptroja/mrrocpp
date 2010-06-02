/*
 * pb_visual_servo.cc
 *
 *  Created on: May 26, 2010
 *      Author: mboryn
 */

#include "pb_visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

pb_visual_servo::pb_visual_servo(boost::shared_ptr <visual_servo_regulator> regulator):visual_servo(regulator)
{
}

pb_visual_servo::~pb_visual_servo()
{}

}//namespace generator {

}

}

}

/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

visual_servo::visual_servo(boost::shared_ptr <visual_servo_regulator> regulator) :
	regulator(regulator), object_visible(false)
{

}

visual_servo::~visual_servo()
{
}

bool visual_servo::is_object_visible(){
	return object_visible;
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

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
	regulator(regulator)
{

}

visual_servo::~visual_servo()
{
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

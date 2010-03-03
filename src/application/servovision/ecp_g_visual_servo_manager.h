/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef ECP_G_VISUAL_SERVO_MANAGER_H_
#define ECP_G_VISUAL_SERVO_MANAGER_H_

#include "ecp_g_visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

class visual_servo_manager: public mrrocpp::ecp::common::generator::visual_servo
{
public:
	visual_servo_manager();
	virtual ~visual_servo_manager();
};

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_VISUAL_SERVO_MANAGER_H_ */

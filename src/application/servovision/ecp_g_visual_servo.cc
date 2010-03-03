/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "ecp_g_visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

visual_servo::visual_servo(mrrocpp::ecp::common::task::task & _ecp_task, visual_servo_regulator * regulator) :
	generator(_ecp_task), regulator(regulator), logEnabled(true)
{

}

visual_servo::~visual_servo()
{
}

void visual_servo::log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!logEnabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush(stdout);
	va_end(ap);
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

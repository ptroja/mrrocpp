/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef ECP_G_VISUAL_SERVO_H_
#define ECP_G_VISUAL_SERVO_H_

#include "ecp/common/generator/ecp_generator.h"
#include "lib/mrmath/mrmath.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"

#include "visual_servo_regulator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

class visual_servo: public mrrocpp::ecp::common::generator::generator
{
public:

	virtual ~visual_servo();
protected:
	visual_servo(mrrocpp::ecp::common::task::task & _ecp_task, visual_servo_regulator * regulator);
	visual_servo_regulator * regulator;

	/** Is log enabled*/
	bool logEnabled;
	/**
	 * Print message to the console only if logEnabled is set to true.
	 * @param fmt printf-like format
	 */
	void log(const char *fmt, ...);
private:

}; // class visual_servo

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_VISUAL_SERVO_H_ */

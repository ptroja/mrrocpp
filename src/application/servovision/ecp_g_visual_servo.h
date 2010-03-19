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

template <int ERROR_SIZE, int CONTROL_SIZE>
class visual_servo: public mrrocpp::ecp::common::generator::generator
{
public:

	virtual ~visual_servo()
	{
	}
	;
protected:
	visual_servo(mrrocpp::ecp::common::task::task & _ecp_task, visual_servo_regulator <ERROR_SIZE, CONTROL_SIZE> * regulator) :
		generator(_ecp_task), regulator(regulator), logEnabled(true), logDbgEnabled(false)
	{
	}

	visual_servo_regulator <ERROR_SIZE, CONTROL_SIZE> * regulator;

	/** Is log enabled*/
	bool logEnabled, logDbgEnabled;

	/**
	 * Print message to the console only if logEnabled is set to true.
	 * @param fmt printf-like format
	 */
	void log(const char *fmt, ...)
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

	/**
	 * Print message to the console only if logDbgEnabled is set to true.
	 * @param fmt printf-like format
	 */
	void logDbg(const char *fmt, ...)
	{
		va_list ap;
		va_start(ap, fmt);

		if (!logDbgEnabled) {
			va_end(ap);
			return;
		}

		vfprintf(stdout, fmt, ap);
		fflush(stdout);
		va_end(ap);
	}
private:

}; // class visual_servo

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_VISUAL_SERVO_H_ */

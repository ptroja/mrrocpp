/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_H_
#define VISUAL_SERVO_H_

#include "lib/mrmath/mrmath.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"
#include <boost/shared_ptr.hpp>
#include "visual_servo_regulator.h"

#include "logger.h"
#include "ecp_mp/sensor/ecp_mp_sensor.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 * Base class for visual servoing.
 * Implements two essential methods: first_step() and next_step(). Subclasses must implement method step() which is called from next_step().
 */
class visual_servo
{
public:
	virtual ~visual_servo();
	/**
	 * Calculates relative change of position.
	 * @param currentPosition
	 * @param newPosition
	 */
	virtual lib::Homog_matrix get_position_change(const lib::Homog_matrix& current_position, double dt) = 0;
	/**
	 * Returns fradia_sensor.
	 * @return
	 */
	virtual boost::shared_ptr <mrrocpp::lib::sensor> get_vsp_fradia() = 0;

	/**
	 * Returns object visibility.
	 * @return
	 */
	virtual bool is_object_visible();
protected:
	visual_servo(boost::shared_ptr <visual_servo_regulator> regulator);
	boost::shared_ptr <visual_servo_regulator> regulator;
	bool object_visible;
private:

}; // class visual_servo

/** @} */

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* VISUAL_SERVO_H_ */

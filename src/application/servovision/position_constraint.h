/*
 * position_constraint.h
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#ifndef POSITION_CONSTRAINT_H_
#define POSITION_CONSTRAINT_H_

#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 * Base class for position constraints in visual servoing.
 * How it works. First we have to set new position by calling set_new_position().
 * After that we check if end effector is in allowed area - call is_translation_ok().
 * If it is, then we check if rotation is ok by calling is_rotation_ok().
 * If it is not, then we call apply_constraint() in order to correct constraint.
 * If translation is not ok, we call apply_constraint() in order to move end effector to allowed area.
 */
class position_constraint
{
public:
	position_constraint();
	virtual ~position_constraint();

	/**
	 * Constraints end effector position.
	 * @param current_position
	 * @return
	 */
	virtual lib::Homog_matrix apply_constraint(const lib::Homog_matrix& current_position) = 0;
protected:
	/**
	 * Makes angle to keep within [-M_PI; M_PI)
	 * @param angle any angle
	 * @return angle within [-M_PI; M_PI)
	 */
	double normalize_angle(double angle);

	/**
	 * Check if angle is within specified range
	 * @param angle
	 * @param min
	 * @param max
	 * @return true if angle in [min; max]
	 */
	bool is_angle_between(double angle, double min, double max);

	/**
	 * Constraints angle so that it's in [min; max]
	 *
	 * @param angle
	 * @param min angle in (-M_PI; M_PI]
	 * @param max angle in (-M_PI; M_PI]
	 * @return angle in [min; max]
	 */
	double constrain_angle(double angle, double min, double max);
private:

};

/** @} */

} // namespace generator
}
}

#endif /* POSITION_CONSTRAINT_H_ */

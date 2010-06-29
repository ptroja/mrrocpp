/*
 * position_constraint.h
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#ifndef POSITION_CONSTRAINT_H_
#define POSITION_CONSTRAINT_H_

#include "lib/mrmath/mrmath.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

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
	 * Update end effector's position.
	 * @param new_position
	 */
	void set_new_position(const lib::Homog_matrix& new_position);

	/**
	 * Check if end effector is in allowed area.
	 * @return
	 */
	virtual bool is_translation_ok() = 0;

	/**
	 * Check if end effector has orientation within limits. This only makes sense, when end effector is in allowed area.
	 * @return
	 */
	virtual bool is_rotation_ok() = 0;

	/**
	 * Get distance from nearest edge of allowed area.
	 * @return
	 */
	virtual double get_distance_from_allowed_area() = 0;

	/**
	 * Constraints end effector position.
	 */
	virtual void apply_constraint() = 0;

	/**
	 * Get end effector's position after constraints are applied.
	 * @return
	 */
	const lib::Homog_matrix& get_constrained_position();
protected:
	lib::Homog_matrix new_position;

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

}

#endif /* POSITION_CONSTRAINT_H_ */

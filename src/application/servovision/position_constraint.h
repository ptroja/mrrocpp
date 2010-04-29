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
 * Abstract method apply_constraint() is to be implemented in subclasses.
 */
class position_constraint
{
public:
	position_constraint();
	virtual ~position_constraint();
	/**
	 * Constraints end effector position.
	 * @param new_position End effector position calculated by generator.
	 * @param constrainted_position Constrained position.
	 */
	virtual void apply_constraint(lib::Homog_matrix& new_position) = 0;

	/**
	 * Check if position is inside allowed region.
	 * @param new_position
	 * @return
	 */
	virtual bool is_position_ok(const lib::Homog_matrix& new_position) = 0;
protected:

private:

};

/** @} */

} // namespace generator

}

}

}

#endif /* POSITION_CONSTRAINT_H_ */

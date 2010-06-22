/*
 * cubic_constraint.h
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#ifndef CUBIC_CONSTRAINT_H_
#define CUBIC_CONSTRAINT_H_

#include "position_constraint.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 * Keeps end effector inside cube of specified dimensions.
 */
class cubic_constraint : public mrrocpp::ecp::common::generator::position_constraint
{
public:
	/**
	 * Generate constraint.
	 * Make sure that translation and rotation vectors are between min(i, 0) and  max(i, 0) for i=0..2
	 * @param translation_min
	 * @param translation_max
	 * @param rotation_min
	 * @param rotation_max
	 */
			cubic_constraint(const Eigen::Matrix <double, 3, 1>& translation_min, const Eigen::Matrix <double, 3, 1>& translation_max, const Eigen::Matrix <
					double, 3, 1>& rotation_min, const Eigen::Matrix <double, 3, 1>& rotation_max);

	virtual ~cubic_constraint();

	virtual bool is_translation_ok();

	virtual bool is_rotation_ok();

	virtual double get_distance_from_allowed_area();

	virtual void apply_constraint();

private:
	Eigen::Matrix <double, 3, 1> translation_min;
	Eigen::Matrix <double, 3, 1> translation_max;
	Eigen::Matrix <double, 3, 1> rotation_min;
	Eigen::Matrix <double, 3, 1> rotation_max;
	Eigen::Matrix <double, 3, 1> rotation_division;
};

/** @} */

} // namespace generator

}

}

}

#endif /* CUBIC_CONSTRAINT_H_ */

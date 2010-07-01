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
class cubic_constraint: public mrrocpp::ecp::common::generator::position_constraint
{
public:
	/**
	 * Generate constraint.
	 * Make sure that p1(i, 0) < p2(i, 0) for i=0..2
	 * @param p1
	 * @param p2
	 * @return
	 */
	cubic_constraint(const Eigen::Matrix<double, 3, 1>& p1, const Eigen::Matrix<double, 3, 1>& p2);
	virtual ~cubic_constraint();
	virtual void apply_constraint(lib::Homog_matrix& new_position);
	virtual bool is_position_ok(const lib::Homog_matrix& new_position);
private:
	Eigen::Matrix<double, 3, 1> p1;
	Eigen::Matrix<double, 3, 1> p2;
};

/** @} */

} // namespace generator

}

}

}

#endif /* CUBIC_CONSTRAINT_H_ */

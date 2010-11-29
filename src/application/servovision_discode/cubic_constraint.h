/*
 * cubic_constraint.h
 *
 *  Created on: Apr 26, 2010
 *      Author: mboryn
 */

#ifndef CUBIC_CONSTRAINT_H_
#define CUBIC_CONSTRAINT_H_

#include <string>
#include "base/lib/configurator.h"

#include "position_constraint.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 * Keeps end effector inside cube of specified dimensions.
 */
class cubic_constraint : public position_constraint
{
public:
	/**
	 * Generate constraint.
	 * Make sure that translation and rotation vectors are between min(i, 0) and  max(i, 0) for i=0..2
	 * In config file in specified section there must be:
	 *    lib::Homog_matrix cube_position;
	 *    Eigen::Matrix <double, 3, 1> cube_size;
	 *    lib::Homog_matrix spherical_cone_rotation;
	 *    double min_inclination;
	 *    double min_wrist_rotation;
	 *    double max_wrist_rotation
	 * @param config
	 * @param section_name
	 * @return
	 */
	cubic_constraint(const lib::configurator& config, const std::string &section_name);

	virtual ~cubic_constraint();

	/**
	 * Constraints end effector position.
	 * @param current_position
	 * @return
	 */
	virtual lib::Homog_matrix apply_constraint(const lib::Homog_matrix& current_position);
protected:
	/**
	 * Make rotation vector fit into spherical_cone specified in config.
	 * @param current_position
	 * @return
	 */
	lib::Homog_matrix constrain_rotation(const lib::Homog_matrix& current_position);

private:
	/** Cube's position with respect to robot's base */
	lib::Homog_matrix cube_position;

	/** Cube's size along X, Y, Z axes */
	Eigen::Matrix <double, 3, 1> cube_size;

	/** For rotation constraint. */
	lib::Homog_matrix spherical_cone_rotation;
	/** For rotation constraint. */
	double min_inclination;
	/** For rotation constraint. */
	double wrist_rotation_min;
	/** For rotation constraint. */
	double wrist_rotation_max;
};

/** @} */

} // namespace generator

}

}

#endif /* CUBIC_CONSTRAINT_H_ */

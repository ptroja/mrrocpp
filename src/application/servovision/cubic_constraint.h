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
	 *    Eigen::Matrix <double, 3, 1> translation_min;
	 *    Eigen::Matrix <double, 3, 1> translation_max;
	 *    Eigen::Matrix <double, 3, 1> cone_axis;
	 *    double min_inclination;
	 *    double min_wrist_rotation;
	 *    double max_wrist_rotation
	 * @param config
	 * @param section_name
	 * @return
	 */
	cubic_constraint(const lib::configurator& config, const std::string &section_name);

	virtual ~cubic_constraint();

	virtual bool is_translation_ok();

	virtual bool is_rotation_ok();

	virtual double get_distance_from_allowed_area();

	virtual void apply_constraint();

private:
	Eigen::Matrix <double, 3, 1> translation_min;
	Eigen::Matrix <double, 3, 1> translation_max;
	//Eigen::Matrix <double, 3, 3> cone_rotation;
	lib::Homog_matrix cone_rotation;
	double min_inclination;
	double wrist_rotation_min;
	double wrist_rotation_max;
};

/** @} */

} // namespace generator

}

}

#endif /* CUBIC_CONSTRAINT_H_ */

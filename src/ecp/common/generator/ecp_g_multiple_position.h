/*
 * multiple_position.h
 *
 *  Created on: May 21, 2010
 *      Author: rtulwin
 */

#ifndef _MULTIPLE_POSITION_H_
#define _MULTIPLE_POSITION_H_

#include "ecp_mp/trajectory_pose/trajectory_pose.h"
#include "ecp/common/generator/ecp_generator.h"

#include <vector>

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * Base class for the motion generators interpolating between fixed trajectory points.
 */
class multiple_position : public generator {
protected:
	/**
	 * Vector of positions (vector of velocity profiles).
	 */
	vector<ecp_mp::common::trajectory_pose::trajectory_pose> pose_vector;
	/**
	 * Position vector iterator.
	 */
	vector<ecp_mp::common::trajectory_pose::trajectory_pose>::iterator pose_vector_iterator;
	/**
	 * Vector of coordinates.
	 */
	vector<vector<double> > coordinate_vector;
	/**
	 * Coordinate vector iterator.
	 */
	vector<vector<double> >::iterator coordinate_vector_iterator;
	/**
	 * Number of axes in which we want to move in the given representation.
	 */
	int axes_num;
	/**
	 * Type of the commanded motion (absolute or relative)
	 */
	lib::MOTION_TYPE motion_type;

public:
	/**
	 * Constructor.
	 */
	multiple_position(common::task::task& _ecp_task);
	/**
	 * Destructor.
	 */
	virtual ~multiple_position();
	/**
	 * Sets the number of axes in which the generator will move the robot.
	 */
	void set_axes_num(int axes_num);
	/**
	 * Sets the chosen type of interpolation.
	 */
	void set_interpolation_type();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _MULTIPLE_POSITION_H_ */

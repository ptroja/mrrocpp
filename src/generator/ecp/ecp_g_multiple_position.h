/*
 * multiple_position.h
 *
 *  Created on: May 21, 2010
 *      Author: rtulwin
 */

#ifndef _MULTIPLE_POSITION_H_
#define _MULTIPLE_POSITION_H_

#include "lib/trajectory_pose/trajectory_pose.h"
#include "generator/ecp/ecp_g_get_position.h"
#include "base/ecp/ecp_generator.h"
#include "generator/ecp/velocity_profile_calculator/velocity_profile.h"

#include <vector>

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * Base class for the motion generators interpolating between fixed trajectory points.
 */
template <class T>
class multiple_position : public generator {
protected:

	/**
	 * Vector of positions (vector of velocity profiles).
	 */
	//vector<ecp_mp::common::trajectory_pose::trajectory_pose> pose_vector;
	vector<T> pose_vector;
	/**
	 * Position vector iterator.
	 */
	typename vector<T>::iterator pose_vector_iterator;
	/**
	 * Vector of coordinates.
	 */
	vector<vector<double> > coordinate_vector;
	/**
	 * Coordinate vector iterator.
	 */
	vector<vector<double> >::iterator coordinate_vector_iterator;
	/**
	 * Type of the commanded motion (absolute or relative)
	 */
	lib::MOTION_TYPE motion_type;
	/**
	 * Velocity profile calculator.
	 */
	velocity_profile_calculator::velocity_profile vpc;
	/**
	 * Number of axes for a given robot in used representation.
	 */
	int axes_num;
	/**
	 * Type of the used representation.
	 */
	lib::ECP_POSE_SPECIFICATION pose_spec;

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
	 * Performs calculation of the trajectory and interpolation. Fills in pose_vector and coordinate_vector.
	 */
	virtual bool calculate_interpolate() = 0;
	/**
	 * Sets the number of axes in which the generator will move the robot.
	 */
	void set_axes_num(int axes_num);
	/**
	 * Sets the chosen type of interpolation.
	 */
	void set_interpolation_type();
	/**
	 * Sets the relative type of motion.
	 */
	void set_relative(void); //zmiana na tryb przyrostowy
	/**
	 * Sets the absolute type of motion.
	 */
	void set_absolute(void); //zmiana na tryb bezwzgledny
	/**
	 * Loads a single trajectory pose described in joint coordinates to the list. Maximal velocities are set automatically.
	 */
	virtual bool load_absolute_joint_trajectory_pose(vector<double> & coordinates) = 0;
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _MULTIPLE_POSITION_H_ */

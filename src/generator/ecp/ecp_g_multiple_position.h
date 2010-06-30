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
#include "generator/ecp/trajectory_interpolator/trajectory_interpolator.h"
#include "lib/mrmath/mrmath.h"

#include <vector>

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * Base class for the motion generators interpolating between fixed trajectory points.
 */
template <class Pos, class Inter, class Calc>
class multiple_position : public generator {
protected:

	/**
	 * Vector of positions (vector of velocity profiles).
	 */
	vector<Pos> pose_vector;
	/**
	 * Position vector iterator.
	 */
	typename vector<Pos>::iterator pose_vector_iterator;
	/**
	 * Vector of coordinates.
	 */
	vector<vector<double> > coordinate_vector;
	/**
	 * Coordinate vector iterator.
	 */
	vector<vector<double> >::iterator coordinate_vector_iterator;
	/**
	 * Temporary iterator used mainly to iterate through a single position which is always of type vector<double>.
	 */
	vector<double>::iterator tempIter;
	/**
	 * Type of the commanded motion (absolute or relative)
	 */
	lib::MOTION_TYPE motion_type;
	/**
	 * Velocity profile calculator.
	 */
	Calc vpc;
	/**
	 * Trajectory interpolator.
	 */
	Inter inter;
	/**
	 * Number of axes for a given robot in used representation.
	 */
	int axes_num;
	/**
	 * Type of the used representation.
	 */
	lib::ECP_POSE_SPECIFICATION pose_spec;
	/**
	 * Set to true if trajectory was calculated (list of positions contains all of the information needed to start the interpolation).
	 */
	bool calculated;
	/**
	 * Set to true if list of coordinates was filled in.
	 */
	bool interpolated;
	/**
	 * Time of a single macrostep.
	 */
	double mc;
	/**
	 * Number of steps in a single macrostep.
	 */
	int nmc;
	/**
	 * If true, debug information is shown.
	 */
	bool debug;
	/**
	 * Standard velocity in joint coordinates.
	 */
	vector<double> joint_velocity;
	/**
	 * Maximal velocity in joint coordinates.
	 */
	vector<double> joint_max_velocity;
	/**
	 * Standard velocity in motor coordinates.
	 */
	vector<double> motor_velocity;
	/**
	 * Maximal velocity in motor coordinates.
	 */
	vector<double> motor_max_velocity;
	/**
	 * Standard velocity in euler zyz coordinates.
	 */
	vector<double> euler_zyz_velocity;
	/**
	 * Maximal velocity in euler zyz coordinates.
	 */
	vector<double> euler_zyz_max_velocity;
	/**
	 * Standard velocity in angle axis coordinates.
	 */
	vector<double> angle_axis_velocity;
	/**
	 * Maximal velocity in angle axis coordinates.
	 */
	vector<double> angle_axis_max_velocity;

public:
	/**
	 * Constructor.
	 */
	multiple_position(common::task::task& _ecp_task) : generator (_ecp_task) {
		debug = false;
	}
	/**
	 * Destructor.
	 */
	virtual ~multiple_position() {

	}
	/**
	 * Set debug variable.
	 */
	void set_debug(bool debug) {
		this->debug = debug;
	}
	/**
	 * Performs calculation of the trajectory and interpolation. Fills in pose_vector and coordinate_vector.
	 * @return true if the calculation was succesfull
	 */
	virtual bool calculate_interpolate() = 0;
	/**
	 * Sets the number of axes in which the generator will move the robot.
	 */
	virtual void set_axes_num(int axes_num) {
		this->axes_num = axes_num;
	}
	/**
	 * Sets the chosen type of interpolation.
	 */
	void set_interpolation_type() {

	}
	/**
	 * Sets the relative type of motion.
	 */
	void set_relative(void) {
		motion_type=lib::RELATIVE;
	}
	/**
	 * Sets the absolute type of motion.
	 */
	void set_absolute(void) {
		motion_type=lib::ABSOLUTE;
	}
	/**
	 * Clears vectors of positions and coordinates. Sets %calculated and %interpolated flags to false;
	 */
	void reset() {
		pose_vector.clear();
		coordinate_vector.clear();
		calculated = false;
		interpolated = false;
		//TODO message "generator reset"
	}
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _MULTIPLE_POSITION_H_ */

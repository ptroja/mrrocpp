/*
 * velocity_profile.h
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#ifndef _VELOCITY_PROFILE_H_
#define _VELOCITY_PROFILE_H_

//#include <list>
#include <math.h>
#include <algorithm>
#include <vector>

#include "lib/trajectory_pose/trajectory_pose.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

/**
 * Base class for all of the velocity profile calculators. Usually any velocity profile calculator contains methods used to create the description
 * of the velocity profile f.g. bang bang velocity profile etc.. This information is usually stored in the appropriate trajectory_pose class.
 */
template <class Pos>
class velocity_profile {
	public:
		/**
		 * Constructor.
		 */
		velocity_profile() {

		}
		/**
		 * Destructor.
		 */
		virtual ~velocity_profile() {

		}
		/**
		 * Method comparing two double values.
		 * @return true if values are the same
		 */
		bool eq(double a, double b) {
			const double EPS = 0.0001;
			const double& diff = a - b;
			return diff < EPS && diff > -EPS;
		}
		/**
		 * Calculates distance for all of the axes in a single trajectory pose and sets the directions of movements of absolute type.
		 * @param it iterator to the list of positions
		 * @return true if the set of the distance and direction was successful (usually is if the vectors start_position and coordinates were initiated and filled in before)
		 */
		bool calculate_absolute_distance_direction_pose(typename vector<Pos>::iterator & it) {

			if (it->coordinates.size() < it->axes_num || it->start_position.size() < it->axes_num) {
				return false;
			}

			it->s.clear();
			it->k.clear();
			for (int i = 0; i < it->axes_num; i++) {
				it->s.push_back(fabs(it->coordinates[i] - it->start_position[i]));
				if (it->coordinates[i] - it->start_position[i] >= 0) {
					it->k.push_back(1);
				} else {
					it->k.push_back(-1);
				}
			}

			return true;
		}
		/**
		 * Calculates distance for all of the axes in a single trajectory pose and sets the directions of movements of relative type..
		 * @param it iterator to the list of positions
		 * @return true if the set of the distance and direction was successful (usually is if the coordinates vector was initiated and filled in before)
		 */
		bool calculate_relative_distance_direction_pose(typename vector<Pos>::iterator & it) {

			if (it->coordinates.size() < it->axes_num) {
				return false;
			}

			it->s.clear();
			it->k.clear();
			for (int i = 0; i < it->axes_num; i++) {
				it->s.push_back(fabs(it->coordinates[i]));
				if (it->coordinates[i] >= 0) {
					it->k.push_back(1);
				} else {
					it->k.push_back(-1);
				}
			}

			return true;
		}
		/**
		 * Calculates time for the given velocity and distance for a single axis in a single pose.
		 * @param it iterator to the list of positions
		 * @param i number of axis for which the calculations are performed
		 * @return true if the time was calculated successfully (if all of the necessary information was provided)
		 */
		virtual bool calculate_time(typename vector<Pos>::iterator & it, int i) = 0;
		/**
		 * Calculates time for the given velocity and distance for all axes in a single pose.
		 * @param it iterator to the list of positions
		 * @return true if the time was calculated successfully (if all of the necessary information was provided)
		 */
		bool calculate_time_pose(typename vector<Pos>::iterator & it) {

			bool trueFlag = true;

			for (int i = 0; i < it->axes_num; i++) {
				if (calculate_time(it, i) == false) {
					trueFlag = false;
				}
			}

			return trueFlag;
		}
		/**
		 * Calculates the longest time from the times vector and stores it in t variable in the pose. Extends t to make it the multiplicity of the macrostep time.
		 * @param it iterator to the list of positions
		 * @param mc macrostep time
		 * @return true if the set of the time was successful (usually is if the vector times was initiated and filled in before)
		 */
		bool calculate_pose_time(typename vector<Pos>::iterator & it, const double & mc) {
			if (it->times.size() == it->axes_num) {
				double t_max = *max_element(it->times.begin(), it->times.end());

				if (t_max == 0) {
					it->t = 0;
					return true;
				}

				if (ceil(t_max / mc) * mc != t_max) { //extend the pose time to be the multiplicity of the macrostep time
					t_max = ceil(t_max / mc);
					t_max = t_max * mc;
					it->t = t_max;
				}

				return true;
			} else {
				return false;
			}
		}
};

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _VELOCITY_PROFILE_H_ */

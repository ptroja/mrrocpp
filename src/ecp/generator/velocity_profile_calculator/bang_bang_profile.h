/*
 * bang_bang_profile.h
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#ifndef _BANG_BANG_PROFILE_H_
#define _BANG_BANG_PROFILE_H_

#include <list>

#include "ecp_mp/smooth_trajectory_pose.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class bang_bang_profile {
	public:
		/**
		 * Constructor.
		 */
		bang_bang_profile();
		/**
		 * Destructor.
		 */
		virtual ~bang_bang_profile();
		/**
		 * Reduces velocity and acceleration of the first motion model velocity profile, so that the given distance is covered in a given time.
		 * Can call reduction_model_2, reduction_model_3 or reduction_model_4 methods if it is necessary.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 */
		void reduction_model_1(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s);
		/**
		 * Reduces velocity and acceleration of the second motion model velocity profile, so that the given distance is covered in a given time.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 */
		void reduction_model_2(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s);
		/**
		 * Reduces velocity and acceleration of the third motion model velocity profile, so that the given distance is covered in a given time.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 */
		void reduction_model_3(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s);
		/**
		 * Reduces velocity and acceleration of the fourth motion model velocity profile, so that the given distance is covered in a given time.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 */
		void reduction_model_4(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s);
		/**
		 * Reduces the terminal velocity so that the given distance can be covered in the given time.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 * @param t time of execution of the given trajectory segment
		 */
		void vk_reduction(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s, double t);
		/**
		 * Reduces the initial velocity so that the given distance can be covered in the given time. Calculated initial velocity is the one at which
		 * the given distance is covered in the given time keeping the velocity constant.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 * @param t time of execution of the given trajectory segment
		 * @return false if the initial values of the velocities were changed and trajectory needs to be recalculated from the beginning
		 */
		bool vp_reduction(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s, double t);
		/**
		 * Calculates the shortest time for the first motion model velocity profile (2 phase motion), in which the given distance can be covered
		 * with the given constraints on maximal velocity and acceleration.
		 * Can call vp_reduction or vk_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 */
		void optimize_time1(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s);
		/**
		 * Calculates the shortest time for the second motion model velocity profile (1 phase motion, only acceleration), in which the given distance
		 * can be covered with the given constraints on maximal velocity and acceleration.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 */
		void optimize_time2(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s);
		/**
		 * Calculates the shortest time for the fourth motion model velocity profile (1 phase motion, only deceleration), in which the given distance
		 * can be covered with the given constraints on maximal velocity and acceleration.
		 * @param i number of axis for which the calculations are performed
		 * @param s distance to be covered in a given axis
		 */
		void optimize_time4(std::list<ecp_mp::common::smooth_trajectory_pose>::iterator &pose_list_iterator, int i, double s);
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _BANG_BANG_PROFILE_H_ */

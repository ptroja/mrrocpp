/*
 * bang_bang_profile.h
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#ifndef _BANG_BANG_PROFILE_H_
#define _BANG_BANG_PROFILE_H_

#include "lib/trajectory_pose/bang_bang_trajectory_pose.h"
#include "generator/ecp/velocity_profile_calculator/velocity_profile.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

class bang_bang_profile : public velocity_profile<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose> {
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
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool reduction_model_1(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Reduces velocity and acceleration of the second motion model velocity profile, so that the given distance is covered in a given time.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool reduction_model_2(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Reduces velocity and acceleration of the third motion model velocity profile, so that the given distance is covered in a given time.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool reduction_model_3(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Reduces velocity and acceleration of the fourth motion model velocity profile, so that the given distance is covered in a given time.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool reduction_model_4(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Reduces the terminal velocity so that the given distance can be covered in the given time.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool vk_reduction(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Reduces the initial velocity so that the given distance can be covered in the given time. Calculated initial velocity is the one at which
		 * the given distance is covered in the given time keeping the velocity constant.
		 * Can call vp_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool vp_reduction(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Calculates the shortest time for the first motion model velocity profile (2 phase motion), in which the given distance can be covered
		 * with the given constraints on maximal velocity and acceleration.
		 * Can call vp_reduction or vk_reduction method.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool optimize_time1(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Calculates the shortest time for the second motion model velocity profile (1 phase motion, only acceleration), in which the given distance
		 * can be covered with the given constraints on maximal velocity and acceleration.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool optimize_time2(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Calculates the shortest time for the fourth motion model velocity profile (1 phase motion, only deceleration), in which the given distance
		 * can be covered with the given constraints on maximal velocity and acceleration.
		 * @param i number of axis for which the calculations are performed
		 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
		 */
		bool optimize_time4(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator &it, int i);
		/**
		 * Calculates time for the given velocity and distance for a single axis in a single pose.
		 * @param it iterator to the list of positions
		 * @param i number of axis for which the calculations are performed
		 * @return true if the time was calculated successfully (if all of the necessary information was provided)
		 */
		bool calculate_time(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
		/**
		 * Sets the v_k (maximal velocity of the next pose, assumed terminal velocity of the current pose) of a single axis of a single pose.
		 * @param it iterator to the list of positions
		 * @param end_it iterator to the one past last element in the pose list
		 * @param i number of axis for which the setting is made
		 * @return true if the calculation was successful
		 */
		bool set_v_k(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & end_it, int i);
		/**
		 * Sets the v_k (maximal velocity of the next pose, assumed terminal velocity of the current pose) of a single pose and all axes.
		 * @param it iterator to the list of positions
		 * @param end_it iterator to the one past last element in the pose list
		 * @return true if the calculation was successful
		 */
		bool set_v_k_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & end_it);
		/**
		 * Sets the v_p (initial velocity) of a single pose and all axes.
		 * @param it iterator to the list of positions
		 * @param beginning_it iterator to the one past last element in the pose list
		 * @return true if the calculation was successful
		 */
		bool set_v_p_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & beginning_it);
		/**
		 * Sets the v_p (initial velocity) of a single axis of a single pose.
		 * @param it iterator to the list of positions
		 * @param beginning_it iterator to the one past last element in the pose list
		 * @param i number of axis for which the setting is made
		 * @return true if the calculation was successful
		 */
		bool set_v_p(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & beginning_it, int i);
		/**
		 * Sets the model of a single axis of a single pose.
		 * @param it iterator to the list of positions
		 * @param i number of axis for which the setting is made
		 * @return true if the calculation was successful
		 */
		bool set_model(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
		/**
		 * Sets the motion model of a single pose and all axes.
		 * @param it iterator to the list of positions
		 * @return true if the calculation was successful
		 */
		bool set_model_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);
		/**
		 * Method checks if s_acc + s_dec is smaller than the whole distance to be covered in a single axis of a single pose.
		 * @param it iterator to the list of positions
		 * @param i number of axis for which the setting is made
		 * @return true if s_acc + s_decc is smaller than s
		 */
		bool check_s_acc_s_decc(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
		/**
		 * Calculates the distances covered in the non uniform motion phases (acceleration and deceleration) for a single axis in a single pose.
		 * @param it iterator to the list of positions
		 * @param i number of axis for which the setting is made
		 * @return true if the calculation was successful
		 */
		bool calculate_s_acc_s_dec(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
		/**
		 * Calculates the distances covered in the non uniform motion phases (acceleration and deceleration) for a single pose.
		 * @param it iterator to the list of positions
		 * @return true if the calculation was successful
		 */
		bool calculate_s_acc_s_dec_pose(vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);

};

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _BANG_BANG_PROFILE_H_ */

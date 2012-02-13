/**
 * @file
 * @brief Contains declarations of the methods of bang_bang_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _BANG_BANG_PROFILE_H_
#define _BANG_BANG_PROFILE_H_

#include "base/lib/trajectory_pose/bang_bang_trajectory_pose.h"
#include "generator/lib/velocity_profile_calculator/velocity_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

/**
 * @brief Class contains methods used to create and calculate the description of the smooth motion of the robot.
 *
 * This velocity profile calculator is used to fill in the objects of trajectory_pose type in the way that they describe the smooth motion of the robot.
 * Methods implemented here can cope with the absolute and relative type of motion.
 */
class bang_bang_profile : public velocity_profile <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>
{
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
	 * @param it iterator to the list of positions
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool reduction_model_1(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Reduces velocity and acceleration of the second motion model velocity profile, so that the given distance is covered in a given time.
	 * Can call vp_reduction method.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the calculations are performed
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool reduction_model_2(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Reduces velocity and acceleration of the third motion model velocity profile, so that the given distance is covered in a given time.
	 * Can call vp_reduction method.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the calculations are performed
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool reduction_model_3(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Reduces velocity and acceleration of the fourth motion model velocity profile, so that the given distance is covered in a given time.
	 * Can call vp_reduction method.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the calculations are performed
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool reduction_model_4(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Reduces the terminal velocity so that the given distance can be covered in the given time.
	 * Can call vp_reduction method.
	 * @param i number of axis for which the calculations are performed
	 * @param it iterator to the list of positions
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool vk_reduction(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Reduces the initial velocity so that the given distance can be covered in the given time. Calculated initial velocity is the one at which
	 * the given distance is covered in the given time keeping the velocity constant.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the calculations are performed
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool vp_reduction(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calculates the shortest time for the first motion model velocity profile (2 phase motion), in which the given distance can be covered
	 * with the given constraints on maximal velocity and acceleration.
	 * Can call vp_reduction or vk_reduction method.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the calculations are performed
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool optimize_time1(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calculates the shortest time for the second motion model velocity profile (1 phase motion, only acceleration), in which the given distance
	 * can be covered with the given constraints on maximal velocity and acceleration.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the calculations are performed
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool optimize_time2(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calculates the shortest time for the fourth motion model velocity profile (1 phase motion, only deceleration), in which the given distance
	 * can be covered with the given constraints on maximal velocity and acceleration. Reduces the initial velocity if necessary.
	 * @param i number of axis for which the calculations are performed
	 * @param it iterator to the list of positions
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool optimize_time4(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calculates time for the given velocity and distance for a single axis in a single pose.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the calculations are performed
	 * @return true if the time was calculated successfully (if all of the necessary information was provided)
	 */
	bool calculate_time(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Sets the v_k (initial velocity of the next pose, assumed terminal velocity of the current pose) of a single axis of a single pose.
	 * @param it iterator to the list of positions
	 * @param end_it iterator to the one past last element in the pose list
	 * @param i number of axis for which the setting is made
	 * @return true if the calculation was successful
	 */
	bool set_v_k(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, std::vector <
			ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & end_it, int i);
	/**
	 * Sets the v_k (initial velocity of the next pose, assumed terminal velocity of the current pose) of a single pose and all axes.
	 * @param it iterator to the list of positions
	 * @param end_it iterator to the one past last element in the pose list
	 * @return true if the calculation was successful
	 */
	bool set_v_k_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, std::vector <
			ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & end_it);
	/**
	 * Sets the v_p (initial velocity) of a single pose and all axes.
	 * @param it iterator to the list of positions
	 * @param beginning_it iterator to the first element in the pose list
	 * @return true if the calculation was successful
	 */
	bool set_v_p_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, std::vector <
			ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & beginning_it);
	/**
	 * Sets the v_p (initial velocity) of a single axis of a single pose.
	 * @param it iterator to the list of positions
	 * @param beginning_it iterator to the first element in the pose list
	 * @param i number of axis for which the setting is made
	 * @return true if the calculation was successful
	 */
	bool set_v_p(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, std::vector <
			ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & beginning_it, int i);
	/**
	 * Sets the model of a single axis of a single pose.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the setting is made
	 * @return true if the calculation was successful
	 */
	bool set_model(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Sets the motion model of a single pose and all axes.
	 * @param it iterator to the list of positions
	 * @return true if the calculation was successful
	 */
	bool set_model_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);
	/**
	 * Method checks if s_acc + s_dec is smaller than the whole distance to be covered in a single axis of a single pose.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the setting is made
	 * @return true if s_acc + s_decc is smaller or equal to s
	 */
	bool check_s_acc_s_decc(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calculates the distances covered in the non uniform motion phases (acceleration and deceleration) for a single axis in a single pose.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the setting is made
	 * @return true if the calculation was successful
	 */
	bool calculate_s_acc_s_dec(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calculates the distances covered in the non uniform motion phases (acceleration and deceleration) for a single pose.
	 * @param it iterator to the list of positions
	 * @return true if the calculation was successful
	 */
	bool calculate_s_acc_s_dec_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);
	/**
	 * Calculates the distances covered in the uniform motion phase for a single axis in a single pose.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the setting is made
	 * @return true if the calculation was successful
	 */
	bool calculate_s_uni(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calculates the distances covered in the uniform motion phase for a single pose.
	 * @param it iterator to the list of positions
	 * @return true if the calculation was successful
	 */
	bool calculate_s_uni_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);
	/**
	 * Calculates and sets the numbers of macrosteps in which the first and second parts of the motion in each axis of the pose end.
	 * @param it iterator to the list of positions
	 * @param mc macrostep time
	 * @return true if the calculation was successful
	 */
	bool calculate_acc_uni_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, const double & mc);
	/**
	 * Calculates and sets the numbers of macrosteps in which the first and second parts of the motion in a single axis of the pose.
	 * @param it iterator to the list of positions
	 * @param mc macrostep time
	 * @param i number of axis for which the setting is made
	 * @return true if the calculation was successful
	 */
	bool calculate_acc_uni(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, const double & mc, int i);
	/**
	 * Calls the appropriate reduction method for a single axis in a single pose, depending on the motion model.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the setting is made
	 * @return true if the calculation was successful
	 */
	bool reduction_axis(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calls the appropriate optimize time method for a single axis in a single pose, depending on the motion model.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the setting is made
	 * @return true if the trajectory recalculation is not needed (if initial velocity was not changed)
	 */
	bool optimize_time_axis(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Calculates velocities and accelerations (v_r and a_r) basing on v, v_max, a and a_max std::vectors for all of the axes in a single pose.
	 * @param it iterator to the list of positions
	 * @return true if the calculation was successful
	 */
	bool calculate_v_r_a_r_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);
	/**
	 * Calculates velocities and accelerations (v_r and a_r) basing on v, v_max, a and a_max std::vectors in one axis in a single pose.
	 * @param it iterator to the list of positions
	 * @param i number of axis for which the setting is made
	 * @return true if the calculation was successful
	 */
	bool calculate_v_r_a_r(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it, int i);
	/**
	 * Method is cleans up the various vectors of a single trajectory pose by filling them in with zeros. It is usually used by the methods which cause
	 * the recursive call of %calculate() method to bring the poses back to initial state which is necessary for the proper functioning of other
	 * methods performing reductions and other calculations.
	 * @param it iterator to the list of positions
	 */
	void clean_up_pose(std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);
};

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _BANG_BANG_PROFILE_H_ */

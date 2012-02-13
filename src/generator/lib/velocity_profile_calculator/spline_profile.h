/**
 * @file
 * @brief Contains declarations of the methods of spline_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _SPLINE_PROFILE_H
#define _SPLINE_PROFILE_H

#include "base/lib/trajectory_pose/spline_trajectory_pose.h"
#include "generator/lib/velocity_profile_calculator/velocity_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

/**
 * @brief Class contains methods used to create and calculate the description of the motion of the robot using spline curves.
 *
 * This velocity profile calculator is used to fill in the trajectory pose list in a way that created trajectory describes the motion of the
 * robot using spline curves. Methods implemented here can cope with the absolute and relative type of motion.
 */
class spline_profile : public velocity_profile<ecp_mp::common::trajectory_pose::spline_trajectory_pose> {
        public:
                /**
                 * Constructor.
                 */
                spline_profile();
                /**
                 * Destructor.
                 */
                virtual ~spline_profile();
                /**
                 * Calculates time for the given velocity and distance for a single axis in a single pose.
                 * @param it iterator to the list odouble timef positions
                 * @param i number of axis for which the calculations are performed
                 * @return true if the time was calculated successfully (if all of the necessary information was provided)
                 */
                bool calculate_time(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i);
                /**
                 * Method generates consecutive powers of a number.
                 * @param x number to be powered
                 * @param power maximal power
                 * @param powers vector to be filled in with powers x
                 */
                inline void generatePowers(int power, double x, double * powers);
                /**
                 *
                 * @param it iterator to the list odouble timef positions
                 * @param i number of axis for which the calculations are performed
                 * @param pos1 initial position of the movement
                 * @param pos2 terminal position of the movement
                 */
                bool calculate_linear_coeffs(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i);
                /**
                 *
                 * @param it iterator to the list odouble timef positions
                 * @param i number of axis for which the calculations are performed
                 * @param pos1 initial position of the movement
                 * @param vel1 initial velcity of the movement
                 * @param pos2 terminal position of the movement
                 * @param vel2 terminal velocity of the movement
                 */
                bool calculate_cubic_coeffs(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i);
                /**
                 *
                 * @param it iterator to the list odouble timef positions
                 * @param i number of axis for which the calculations are performed
                 * @param pos1 initial position of the movement
                 * @param vel1 initial velocity of the movement
                 * @param acc1 initial acceleration of the movement
                 * @param pos2 terminal position of the movement
                 * @param vel2 terminal velocity of the movement
                 * @param acc2 terminal acceleration of the movement
                 */
                bool calculate_quintic_coeffs(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, int i);
                /**
                 * Sets the v_p (initial velocity) of a single pose and all axes.
                 * @param it iterator to the list of positions
                 * @param beginning_it iterator to the first element in the pose list
                 * @return true if the calculation was successful
                 */
                bool set_v_p_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & beginning_it);
                /**
                 * Sets the v_p (initial velocity) of a single axis of a single pose.
                 * @param it iterator to the list of positions
                 * @param beginning_it iterator to the first element in the pose list
                 * @param i number of axis for which the setting is made
                 * @return true if the calculation was successful
                 */
                bool set_v_p(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & beginning_it, int i);
                /**
                 * Sets the v_k (initial velocity of the next pose, assumed terminal velocity of the current pose) of a single axis of a single pose.
                 * @param it iterator to the list of positions
                 * @param end_it iterator to the one past last element in the pose list
                 * @param i number of axis for which the setting is made
                 * @return true if the calculation was successful
                 */
                bool set_v_k(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & end_it, int i);
                /**
                 * Sets the v_k (initial velocity of the next pose, assumed terminal velocity of the current pose) of a single pose and all axes.
                 * @param it iterator to the list of positions
                 * @param end_it iterator to the one past last element in the pose list
                 * @return true if the calculation was successful
                 */
                bool set_v_k_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & end_it);
                /**
                 * Sets the a_p (initial acceleration) of a single pose and all axes.
                 * @param it iterator to the list of positions
                 * @param beginning_it iterator to the first element in the pose list
                 * @return true if the calculation was successful
                 */
                bool set_a_p_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & beginning_it);
                /**
                 * Sets the a_p (initial acceleration) of a single axis of a single pose.
                 * @param it iterator to the list of positions
                 * @param beginning_it iterator to the first element in the pose list
                 * @param i number of axis for which the setting is made
                 * @return true if the calculation was successful
                 */
                bool set_a_p(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & beginning_it, int i);
                /**
                 * Sets the a_k (initial acceleration of the next pose, terminal acceleration of the current pose) of a single axis of a single pose.
                 * @param it iterator to the list of positions
                 * @param end_it iterator to the one past last element in the pose list
                 * @param i number of axis for which the setting is made
                 * @return true if the calculation was successful
                 */
                bool set_a_k(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & end_it, int i);
                /**
                 * Sets the a_k (initial acceleration of the next pose, terminal acceleration of the current pose) of a single pose and all axes.
                 * @param it iterator to the list of positions
                 * @param end_it iterator to the one past last element in the pose list
                 * @return true if the calculation was successful
                 */
                bool set_a_k_pose(std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & it, std::vector<ecp_mp::common::trajectory_pose::spline_trajectory_pose>::iterator & end_it);
            };

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif // _SPLINE_PROFILE_H

/**
 * @file
 * @brief Contains declarations of the methods of spline class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _ECP_G_SPLINE_H
#define _ECP_G_SPLINE_H

#include "generator/ecp/ecp_g_multiple_position.h"
#include "base/lib/trajectory_pose/spline_trajectory_pose.h"
#include "generator/ecp/velocity_profile_calculator/spline_profile.h"
#include "generator/ecp/trajectory_interpolator/spline_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief
 *
 * @author rtulwin
 * @ingroup generators
 */
class spline : public multiple_position<ecp_mp::common::trajectory_pose::spline_trajectory_pose,
                                        ecp::common::generator::trajectory_interpolator::spline_interpolator,
                                        ecp::common::generator::velocity_profile_calculator::spline_profile> {
        public:
            /**
             * Constructor. Sets the axes_num and pose_spec variables.
             * @param _ecp_task current ecp task
             * @param axes_num number of axes for a given robot and representation
             * @param pose_spec representation in which the robot position is expressed
             */
            spline(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);

            /**
             * Destructor.
             */
            virtual ~spline();
            /**
             * Method load the absolute trajectory_pose object to the pose_vector.
             * @param trajectory_pose pose to load
             */
            bool load_absolute_pose(ecp_mp::common::trajectory_pose::spline_trajectory_pose & trajectory_pose);
            /**
             * Method load the relative trajectory_pose object to the pose_vector.
             * @param trajectory_pose pose to load
             */
            bool load_relative_pose(ecp_mp::common::trajectory_pose::spline_trajectory_pose & trajectory_pose);
       private:
            /**
             * Creates the vectors containing the information about the maximal and typical velocities for each representation.
             * @param axes_num actual number of axes
             */
            void create_velocity_vectors(int axes_num);
            /**
             * Calculates trajectory.
             * @return true if calculation was successful.
             */
            bool calculate();
            /**
             * Method used to print list of positions.
             */
            void print_pose_vector();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif // _ECP_G_SPLINE_H


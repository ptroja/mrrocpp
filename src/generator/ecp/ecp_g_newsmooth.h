/**
* @file	ecp_g_newsmooth.h
* @brief Smooth class and its methods.
* @author rtulwin
* @date	2010
*/

#if !defined(_ECP_GEN_NEWSMOOTH_H)
# define _ECP_GEN_NEWSMOOTH_H

#include <math.h>

using namespace std;

#include "generator/ecp/ecp_g_multiple_position.h"
#include "lib/trajectory_pose/bang_bang_trajectory_pose.h"
#include "generator/ecp/velocity_profile_calculator/bang_bang_profile.h"
#include "generator/ecp/trajectory_interpolator/bang_bang_interpolator.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * Smooth trajectory generator which has an ability to calculate every trajectory (posiada moce super krowy).
 */
class newsmooth : public multiple_position<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose,
ecp::common::generator::trajectory_interpolator::bang_bang_interpolator,
ecp::common::generator::velocity_profile_calculator::bang_bang_profile> {

	protected:

	public:
		/**
		 * Performs calculation of the trajectory and interpolation. Fills in pose_vector and coordinate_vector.
		 * @return true if the calculation was succesfull
		 */
		bool calculate_interpolate();
		/**
		 * Constructor.
		 */
		newsmooth(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);
		/**
		 * Destructor.
		 */
		virtual ~newsmooth();
		/**
		 * Implementation of the first_step method.
		 */
		bool first_step();
		/**
		 * Implementation of the next_step method.
		 */
		bool next_step();
		/**
		 * Sets the number of axes in which the generator will move the robot. New velocity and acceleration vectors are created to match the new number of axes.
		 */
		void set_axes_num(int axes_num);

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

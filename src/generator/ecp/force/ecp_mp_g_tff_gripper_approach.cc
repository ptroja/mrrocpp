/*!
 * @file
 * @brief File contains ecp mp tff nose run definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "ecp_mp_g_tff_gripper_approach.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {
namespace tff_gripper_approach {

behaviour_specification_data_type::behaviour_specification_data_type(double l_speed, unsigned int l_motion_time, double l_force_level)
{
	speed = l_speed;
	motion_time = l_motion_time;
	force_level = l_force_level;
}

behaviour_specification_data_type::behaviour_specification_data_type()
{
}

}
} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp

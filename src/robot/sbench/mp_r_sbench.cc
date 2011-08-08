/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "robot/sbench/mp_r_sbench.h"

namespace mrrocpp {
namespace mp {
namespace robot {

sbench::sbench(const lib::robot_name_t & l_robot_name, task::task &mp_object_l) :
	mp::robot::robot(lib::sbench::ROBOT_NAME, mp_object_l, lib::sbench::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


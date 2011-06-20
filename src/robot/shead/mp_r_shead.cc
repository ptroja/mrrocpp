/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "robot/shead/mp_r_shead.h"

namespace mrrocpp {
namespace mp {
namespace robot {

shead::shead(const lib::robot_name_t & l_robot_name, task::task &mp_object_l) :
	mp::robot::robot(l_robot_name, mp_object_l, lib::shead::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


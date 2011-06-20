/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "robot/smb/mp_r_smb.h"

namespace mrrocpp {
namespace mp {
namespace robot {

smb::smb(const lib::robot_name_t & l_robot_name, task::task &mp_object_l) :
	mp::robot::robot(l_robot_name, mp_object_l, lib::smb::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


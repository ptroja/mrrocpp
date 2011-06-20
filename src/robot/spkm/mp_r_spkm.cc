/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "mp_r_spkm.h"

namespace mrrocpp {
namespace mp {
namespace robot {

spkm::spkm(const lib::robot_name_t & l_robot_name, task::task &mp_object_l) :
	mp::robot::robot(l_robot_name, mp_object_l, lib::spkm::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "robot/spkm/mp_r_spkm.h"


namespace mrrocpp {
namespace mp {
namespace robot {

spkm::spkm(task::task &mp_object_l) :
	robot(lib::spkm::ROBOT_NAME, lib::spkm::ECP_SECTION, mp_object_l, lib::spkm::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

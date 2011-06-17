/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "mp_r_spkm2.h"

namespace mrrocpp {
namespace mp {
namespace robot {

spkm2::spkm2(task::task &mp_object_l) :
	spkm(lib::spkm2::ROBOT_NAME, mp_object_l)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "base/mp/mp_task.h"

#include "base/lib/swarmtypes.h"
#include "mp_r_spkm.h"

#include "base/lib/agent/InputBuffer.h"

namespace mrrocpp {
namespace mp {
namespace robot {

spkm::spkm(const lib::robot_name_t & l_robot_name, task::task &mp_object_l) :
	mp::robot::robot(l_robot_name, mp_object_l, lib::spkm::NUM_OF_SERVOS),
	notifyBuffer(l_robot_name+"_NOTIFICATION"),
	nextstateBuffer(ecp, l_robot_name+lib::nextstateBufferId),
	notification(notifyBuffer.access)
{
	mp_object_l.registerBuffer(notifyBuffer);
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

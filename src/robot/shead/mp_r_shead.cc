/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "robot/shead/mp_r_shead.h"
#include "robot/shead/const_shead.h"

namespace mrrocpp {
namespace mp {
namespace robot {

shead::shead(task::task &mp_object_l) :
	motor_driven(lib::shead::ROBOT_SHEAD, lib::shead::ECP_SECTION, mp_object_l, lib::shead::SHEAD_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


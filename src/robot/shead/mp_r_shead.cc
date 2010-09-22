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

shead::shead(task::task &mp_object_l) :
	robot(lib::shead::ROBOT_NAME, lib::shead::ECP_SECTION, mp_object_l, lib::shead::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


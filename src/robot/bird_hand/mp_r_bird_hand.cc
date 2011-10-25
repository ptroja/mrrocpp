/*!
 * @file
 * @brief File contains mp robot class definition for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "mp_r_bird_hand.h"

namespace mrrocpp {
namespace mp {
namespace robot {

bird_hand::bird_hand(task::task &mp_object_l) :
	mp::robot::robot(lib::bird_hand::ROBOT_NAME, mp_object_l, lib::bird_hand::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


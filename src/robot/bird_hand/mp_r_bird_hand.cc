/*!
 * @file
 * @brief File contains mp robot class definition for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "mp_r_bird_hand.h"
#include "const_bird_hand.h"

namespace mrrocpp {
namespace mp {
namespace robot {

bird_hand::bird_hand(task::task &mp_object_l) :
	motor_driven(lib::ROBOT_BIRD_HAND, ECP_BIRD_HAND_SECTION, mp_object_l, BIRD_HAND_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


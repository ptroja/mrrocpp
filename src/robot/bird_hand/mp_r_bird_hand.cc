#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp_r_bird_hand.h"

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


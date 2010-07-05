#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/sarkofag/sarkofag_const.h"

namespace mrrocpp {
namespace mp {
namespace robot {

sarkofag::sarkofag(task::task &mp_object_l) :
	motor_driven(lib::ROBOT_SARKOFAG, ECP_SARKOFAG_SECTION, mp_object_l, SARKOFAG_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

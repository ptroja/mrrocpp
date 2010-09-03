#include "robot/polycrank/mp_r_polycrank.h"


namespace mrrocpp {
namespace mp {
namespace robot {

polycrank::polycrank(task::task &mp_object_l) :
			robot(lib::polycrank::ROBOT_NAME, lib::polycrank::ECP_SECTION, mp_object_l, lib::polycrank::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


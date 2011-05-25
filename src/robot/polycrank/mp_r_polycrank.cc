#include "base/mp/MP_main_error.h"
#include "robot/polycrank/mp_r_polycrank.h"

namespace mrrocpp {
namespace mp {
namespace robot {

polycrank::polycrank(task::task &mp_object_l) :
	robot(lib::polycrank::ROBOT_NAME, mp_object_l, lib::polycrank::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


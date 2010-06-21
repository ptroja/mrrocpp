#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "robot/conveyor/mp_r_conveyor.h"

namespace mrrocpp {
namespace mp {
namespace robot {

conveyor::conveyor(task::task &mp_object_l) :
	motor_driven(lib::ROBOT_CONVEYOR, ECP_CONVEYOR_SECTION, mp_object_l, CONVEYOR_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


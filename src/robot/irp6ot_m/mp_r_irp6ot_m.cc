#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6ot_m::irp6ot_m(task::task &mp_object_l) :
	motor_driven(lib::ROBOT_IRP6OT_M, ECP_IRP6OT_M_SECTION, mp_object_l, IRP6OT_M_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


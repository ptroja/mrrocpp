#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_r_conveyor.h"

namespace mrrocpp {
namespace mp {
namespace common {

conveyor_robot::conveyor_robot(task::task &mp_object_l) :
	irp6s_and_conv_robot(lib::ROBOT_CONVEYOR, "[ecp_conveyor]", mp_object_l)
{
}

} // namespace common
} // namespace mp
} // namespace mrrocpp


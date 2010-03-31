#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/robot/mp_r_irp6_postument.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6_postument::irp6_postument (task::task &mp_object_l) :
		manip_and_conv (lib::ROBOT_IRP6_POSTUMENT, ECP_IRP6_POSTUMENT_SECTION, mp_object_l, IRP6_POSTUMENT_NUM_OF_SERVOS), has_gripper(true)
{}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


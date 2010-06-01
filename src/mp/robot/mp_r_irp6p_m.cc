#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/robot/mp_r_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6p_m::irp6p_m(task::task &mp_object_l) :
	manip_and_conv(lib::ROBOT_IRP6P_M, ECP_IRP6P_M_SECTION, mp_object_l,
			IRP6P_M_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


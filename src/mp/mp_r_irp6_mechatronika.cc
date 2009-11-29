#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp_r_irp6_mechatronika.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6_mechatronika::irp6_mechatronika (task::task &mp_object_l) :
		manip_and_conv (lib::ROBOT_IRP6_MECHATRONIKA, ECP_IRP6_MECHATRONIKA_SECTION, mp_object_l)
{}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


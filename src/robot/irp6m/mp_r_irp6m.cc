#include "robot/irp6m/mp_r_irp6m.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6m::irp6m(task::task &mp_object_l) :
	robot(lib::irp6m::ROBOT_NAME, lib::irp6m::ECP_SECTION, mp_object_l, lib::irp6m::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


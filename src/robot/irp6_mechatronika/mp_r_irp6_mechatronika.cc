#include "robot/irp6_mechatronika/mp_r_irp6_mechatronika.h"
#include "robot/irp6_mechatronika/const_irp6m.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6_mechatronika::irp6_mechatronika(task::task &mp_object_l) :
	motor_driven(lib::irp6m::ROBOT_NAME, lib::irp6m::ECP_SECTION, mp_object_l, lib::irp6m::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


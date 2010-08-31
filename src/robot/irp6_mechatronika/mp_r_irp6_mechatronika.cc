#include "robot/irp6_mechatronika/mp_r_irp6_mechatronika.h"
#include "robot/irp6_mechatronika/irp6m_const.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6_mechatronika::irp6_mechatronika(task::task &mp_object_l) :
			motor_driven(lib::irp6m::ROBOT_IRP6_MECHATRONIKA, ECP_IRP6_MECHATRONIKA_SECTION, mp_object_l, IRP6_MECHATRONIKA_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


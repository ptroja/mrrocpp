#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/irp6p_tfg/irp6p_tfg_const.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6p_tfg::irp6p_tfg(task::task &mp_object_l) :
	motor_driven(lib::ROBOT_IRP6P_TFG, ECP_IRP6P_TFG_SECTION, mp_object_l, IRP6P_TFG_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


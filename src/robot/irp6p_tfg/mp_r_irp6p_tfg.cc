/*!
 * @file
 * @brief File contains mp robot class definition for IRp6 postument two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_tfg
 */

#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6p_tfg::irp6p_tfg(task::task &mp_object_l) :
			motor_driven(lib::irp6p_tfg::ROBOT_IRP6P_TFG, lib::irp6p_tfg::ECP_SECTION, mp_object_l, lib::irp6p_tfg::IRP6P_TFG_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


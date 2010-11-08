/*!
 * @file
 * @brief File contains mp robot class definition for IRp6 postument two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_tfg
 */

#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"


namespace mrrocpp {
namespace mp {
namespace robot {

irp6p_tfg::irp6p_tfg(task::task &mp_object_l) :
			robot(lib::irp6p_tfg::ROBOT_NAME, lib::irp6p_tfg::ECP_SECTION, mp_object_l, lib::irp6p_tfg::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


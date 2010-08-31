/*!
 * @file
 * @brief File contains mp robot class definition for IRp6 postument manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_m
 */

#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6p_m::irp6p_m(task::task &mp_object_l) :
	motor_driven(lib::irp6p_m::ROBOT_NAME, lib::irp6p_m::ECP_SECTION, mp_object_l, lib::irp6p_m::IRP6P_M_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


/*!
 * @file
 * @brief File contains mp robot class definition for IRp6 on track manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_m
 */

#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"

namespace mrrocpp {
namespace mp {
namespace robot {

irp6ot_m::irp6ot_m(task::task &mp_object_l) :
	robot(lib::irp6ot_m::ROBOT_IRP6OT_M, lib::irp6ot_m::ECP_SECTION, mp_object_l, lib::irp6ot_m::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


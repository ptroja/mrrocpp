/*!
 * @file mp_r_conveyor.cc
 * @brief File contains mp robot class definition for Conveyor
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup conveyor
 */

#include "base/mp/MP_main_error.h"
#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/conveyor/conveyor_const.h"

namespace mrrocpp {
namespace mp {
namespace robot {

conveyor::conveyor(task::task &mp_object_l) :
	motor_driven(lib::ROBOT_CONVEYOR, ECP_CONVEYOR_SECTION, mp_object_l, CONVEYOR_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp


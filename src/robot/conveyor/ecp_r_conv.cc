/*!
 * @file
 * @brief File contains ecp robot class definition for Conveyor
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup conveyor
 */

#include "base/lib/impconst.h"

#include "robot/conveyor/ecp_r_conv.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	robot::ecp_robot(lib::ROBOT_CONVEYOR, CONVEYOR_NUM_OF_SERVOS, EDP_CONVEYOR_SECTION, _config, _sr_ecp)
{
}
robot::robot(common::task::task& _ecp_object) :
	robot::ecp_robot(lib::ROBOT_CONVEYOR, CONVEYOR_NUM_OF_SERVOS, EDP_CONVEYOR_SECTION, _ecp_object)
{
}

} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp



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
	ecp::common::robot::ecp_robot(lib::conveyor::ROBOT_NAME, lib::conveyor::NUM_OF_SERVOS, _config, _sr_ecp)
{
}
robot::robot(common::task::task_base& _ecp_object) :
	ecp::common::robot::ecp_robot(lib::conveyor::ROBOT_NAME, lib::conveyor::NUM_OF_SERVOS, _ecp_object)
{
}

} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp



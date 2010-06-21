// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - conveyor
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "robot/conveyor/ecp_r_conv.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp_robot(lib::ROBOT_CONVEYOR, CONVEYOR_NUM_OF_SERVOS, EDP_CONVEYOR_SECTION, _config, _sr_ecp)
{
}
robot::robot(common::task::task& _ecp_object) :
	ecp_robot(lib::ROBOT_CONVEYOR, CONVEYOR_NUM_OF_SERVOS, EDP_CONVEYOR_SECTION, _ecp_object)
{
}

} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp



// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "robot/irp6m/ecp_r_irp6m.h"

namespace mrrocpp {
namespace ecp {
namespace irp6m {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	robot::ecp_robot(lib::irp6m::ROBOT_NAME, lib::irp6m::NUM_OF_SERVOS, lib::irp6m::EDP_SECTION, _config, _sr_ecp)
{
}
robot::robot(common::task::task& _ecp_object) :
	robot::ecp_robot(lib::irp6m::ROBOT_NAME, lib::irp6m::NUM_OF_SERVOS, lib::irp6m::EDP_SECTION, _ecp_object)
{
}

} // namespace irp6m
} // namespace ecp
} // namespace mrrocpp


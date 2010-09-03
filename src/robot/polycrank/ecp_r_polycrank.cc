// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "robot/polycrank/ecp_r_polycrank.h"

namespace mrrocpp {
namespace ecp {
namespace polycrank {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
			robot::ecp_robot(lib::polycrank::ROBOT_NAME, lib::polycrank::NUM_OF_SERVOS, lib::polycrank::EDP_SECTION, _config, _sr_ecp)
{
}
;
robot::robot(common::task::task& _ecp_object) :
			robot::ecp_robot(lib::polycrank::ROBOT_NAME, lib::polycrank::NUM_OF_SERVOS, lib::polycrank::EDP_SECTION, _ecp_object)
{
}
;

} // namespace polycrank
} // namespace ecp
} // namespace mrrocpp


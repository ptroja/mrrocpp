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
	ecp_robot(lib::ROBOT_POLYCRANK, POLYCRANK_NUM_OF_SERVOS, EDP_POLYCRANK_SECTION, _config, _sr_ecp)
{
}
;
robot::robot(common::task::task& _ecp_object) :
	ecp_robot(lib::ROBOT_POLYCRANK, POLYCRANK_NUM_OF_SERVOS, EDP_POLYCRANK_SECTION, _ecp_object)
{
}
;

} // namespace polycrank
} // namespace ecp
} // namespace mrrocpp


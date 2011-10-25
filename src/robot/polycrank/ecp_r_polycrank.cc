// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - polycrank
//
// -------------------------------------------------------------------------

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "robot/polycrank/ecp_r_polycrank.h"

//#include "base/lib/impconst.h"
//#include "robot/conveyor/ecp_r_conv.h"

namespace mrrocpp {
namespace ecp {
namespace polycrank {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp::common::robot::ecp_robot(lib::polycrank::ROBOT_NAME, lib::polycrank::NUM_OF_SERVOS, _config, _sr_ecp)
{
}

robot::robot(common::task::task_base& _ecp_object) :
	ecp::common::robot::ecp_robot(lib::polycrank::ROBOT_NAME, lib::polycrank::NUM_OF_SERVOS, _ecp_object)
{
}

} // namespace polycrank
} // namespace ecp
} // namespace mrrocpp


// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mis_fun.h"

#include "robot/irp6_mechatronika/ecp_r_irp6m.h"

namespace mrrocpp {
namespace ecp {
namespace irp6m {

robot::robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp):
	robot::ecp_robot (lib::ROBOT_IRP6_MECHATRONIKA, IRP6_MECHATRONIKA_NUM_OF_SERVOS, EDP_IRP6_MECHATRONIKA_SECTION, _config, _sr_ecp){}
robot::robot (common::task::task& _ecp_object):
	robot::ecp_robot (lib::ROBOT_IRP6_MECHATRONIKA, IRP6_MECHATRONIKA_NUM_OF_SERVOS, EDP_IRP6_MECHATRONIKA_SECTION, _ecp_object){}


} // namespace irp6m
} // namespace ecp
} // namespace mrrocpp


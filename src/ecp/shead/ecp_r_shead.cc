// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/shead/ecp_r_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {

robot::robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp):
	ecp_robot (lib::ROBOT_SHEAD, SMB_NUM_OF_SERVOS, EDP_SMB_SECTION, _config, _sr_ecp){};
robot::robot (common::task::task& _ecp_object):
	ecp_robot (lib::ROBOT_SHEAD, SMB_NUM_OF_SERVOS, EDP_SMB_SECTION, _ecp_object){};


} // namespace smb
} // namespace ecp
} // namespace mrrocpp


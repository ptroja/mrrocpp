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

#include "ecp/spkm/ecp_r_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

robot::robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp):
	ecp_robot (lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION, _config, _sr_ecp){};
robot::robot (common::task::task& _ecp_object):
	ecp_robot (lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION, _ecp_object){};

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp


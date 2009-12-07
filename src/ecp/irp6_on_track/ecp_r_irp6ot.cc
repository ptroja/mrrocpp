// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_on_track
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp_robot(lib::ROBOT_IRP6_ON_TRACK, IRP6_ON_TRACK_NUM_OF_SERVOS, EDP_IRP6_ON_TRACK_SECTION, _config, _sr_ecp)
{
}

robot::robot(common::task::task& _ecp_object) :
	ecp_robot(lib::ROBOT_IRP6_ON_TRACK, IRP6_ON_TRACK_NUM_OF_SERVOS, EDP_IRP6_ON_TRACK_SECTION, _ecp_object)
{
}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp



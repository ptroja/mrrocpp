// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_postument
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/irp6_postument/ecp_r_irp6p.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp_robot(lib::ROBOT_IRP6_POSTUMENT, IRP6_POSTUMENT_NUM_OF_SERVOS, EDP_IRP6_POSTUMENT_SECTION, _config, _sr_ecp)
{
}

robot::robot(common::task::task& _ecp_object) :
	ecp_robot(lib::ROBOT_IRP6_POSTUMENT, IRP6_POSTUMENT_NUM_OF_SERVOS, EDP_IRP6_POSTUMENT_SECTION, _ecp_object)
{
}

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

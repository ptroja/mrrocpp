// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_postument
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_IRP6_POSTUMENT_H)
#define _ECP_R_IRP6_POSTUMENT_H

#include "ecp/common/ecp_robot.h"
#include "lib/irp6p_const.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {

// ---------------------------------------------------------------
class robot: public common::ecp_robot
{
	// Klasa dla robota irp6_postument (sztywnego)

	public:
	robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot (common::task::task& _ecp_object);



}; // end: class ecp_irp6_postument_robot
// ---------------------------------------------------------------

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif

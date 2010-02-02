// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_SMB_H)
#define _ECP_R_SMB_H

#include "ecp/common/ecp_robot.h"
#include "lib/smb_const.h"

namespace mrrocpp {
namespace ecp {
namespace smb {

// ---------------------------------------------------------------
class robot: public common::ecp_robot
{
	// Klasa dla robota irp6_postument (sztywnego)

	public:
	robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot (common::task::task& _ecp_object);



}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif

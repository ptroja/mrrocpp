// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_on_track
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_IRP6_ON_TRACK_H)
#define _ECP_R_IRP6_ON_TRACK_H

#include "ecp/common/ecp_robot.h"
#include "lib/irp6ot_const.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

// ---------------------------------------------------------------
class robot: public common::ecp_robot
{
	// Klasa dla robota irp6_on_track

	public:
	robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot (common::task::task& _ecp_object);



}; // end: class ecp_irp6_on_track_robot
// ---------------------------------------------------------------

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

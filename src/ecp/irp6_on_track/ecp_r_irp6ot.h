// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_on_track
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_IRP6_ON_TRACK_H)
#define _ECP_R_IRP6_ON_TRACK_H

#include "ecp/common/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

// ---------------------------------------------------------------
class robot: public common::ecp_robot
{
	// Klasa dla robota irp6_on_track

	public:
	robot (lib::configurator &_config, lib::sr_ecp *_sr_ecp);
	robot (common::task::task& _ecp_object);

	virtual void create_command (void);
	// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w obrazie
	// robota wykorzystywanych przez generator
	// Ten bufor znajduje sie w robocie

	void get_reply (void);
	virtual void get_input_reply (void);
	virtual void get_arm_reply (void);
	virtual void get_rmodel_reply (void);

}; // end: class ecp_irp6_on_track_robot
// ---------------------------------------------------------------

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

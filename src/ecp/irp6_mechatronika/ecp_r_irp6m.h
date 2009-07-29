// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_IRP6_MECHATRONIKA_H)
#define _ECP_R_IRP6_MECHATRONIKA_H

#include "ecp/common/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6m {

// ---------------------------------------------------------------
class ecp_irp6_mechatronika_robot: public common::ecp_robot
{
	// Klasa dla robota irp6_postument (sztywnego)

	public:
	ecp_irp6_mechatronika_robot (lib::configurator &_config, lib::sr_ecp *_sr_ecp);
	ecp_irp6_mechatronika_robot (common::task::task& _ecp_object);

	virtual void create_command (void);
	// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w obrazie
	// robota wykorzystywanych przez generator
	// Ten bufor znajduje sie w robocie

	void get_reply (void);
	virtual void get_input_reply (void);
	virtual void get_arm_reply (void);
	virtual void get_rmodel_reply (void);

}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace irp6m
} // namespace ecp
} // namespace mrrocpp

#endif

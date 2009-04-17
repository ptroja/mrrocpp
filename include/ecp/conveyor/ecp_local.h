// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - conveyor
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_CONVEYOR_LOCAL_H)
#define _ECP_CONVEYOR_LOCAL_H

#include "ecp/common/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

// ---------------------------------------------------------------
class ecp_conveyor_robot: public common::ecp_robot
{
	// Klasa dla robota conveyor (sztywnego)

	public:
	ecp_conveyor_robot (lib::configurator &_config, lib::sr_ecp *_sr_ecp);
	ecp_conveyor_robot (common::task::task& _ecp_object);

	virtual void create_command (void);
	// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w obrazie
	// robota wykorzystywanych przez generator
	// Ten bufor znajduje sie w robocie

	void get_reply (void);
	virtual void get_input_reply (void);  
	virtual void get_arm_reply (void);
	virtual void get_rmodel_reply (void);

}; // end: class ecp_conveyor_robot
// ---------------------------------------------------------------

} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif

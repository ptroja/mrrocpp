// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_on_track
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_IRP6_ON_TRACK_LOCAL_H)
#define _ECP_IRP6_ON_TRACK_LOCAL_H

#include "ecp/common/ecp_robot.h"

// ---------------------------------------------------------------
class ecp_irp6_on_track_robot: public ecp_robot
{
	// Klasa dla robota irp6_on_track

	public:
	ecp_irp6_on_track_robot (configurator &_config, sr_ecp *_sr_ecp);
	ecp_irp6_on_track_robot (ecp_task& _ecp_object);

	virtual void create_command (void);
	// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w obrazie
	// robota wykorzystywanych przez generator
	// Ten bufor znajduje sie w robocie

	virtual void get_reply (void);
	virtual void get_input_reply (void);  
	virtual void get_arm_reply (void);
	virtual void get_rmodel_reply (void);

}; // end: class ecp_irp6_on_track_robot
// ---------------------------------------------------------------

#endif

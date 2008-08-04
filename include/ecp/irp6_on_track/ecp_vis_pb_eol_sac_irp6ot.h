///////////////////////////////////////////////////////////
//  ecp_vis_pb_eol_sac_irp6ot.h
//  Implementation of the Class ecp_vis_pb_eol_sac_irp6ot
//  Created on:      04-sie-2008 14:25:57
//  Original author: tkornuta
///////////////////////////////////////////////////////////

#if !defined(EA_B10EA6BB_2E03_4f87_8E50_B6482560D9BB__INCLUDED_)
#define EA_B10EA6BB_2E03_4f87_8E50_B6482560D9BB__INCLUDED_

#include "ecp_visual_servo.h"

class ecp_vis_pb_eol_sac_irp6ot : public ecp_visual_servo
{

public:
	ecp_vis_pb_eol_sac_irp6ot();
	virtual ~ecp_vis_pb_eol_sac_irp6ot();

	virtual void next_step_without_constraints()();
	virtual void entertain_constraints();
	virtual bool first_step(void);

};
#endif // !defined(EA_B10EA6BB_2E03_4f87_8E50_B6482560D9BB__INCLUDED_)

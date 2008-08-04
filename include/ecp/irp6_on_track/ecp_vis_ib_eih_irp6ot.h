///////////////////////////////////////////////////////////
//  ecp_vis_ib_eih_irp6ot.h
//  Implementation of the Class ecp_vis_ib_eih_irp6ot
//  Created on:      04-sie-2008 14:26:11
//  Original author: tkornuta
///////////////////////////////////////////////////////////

#if !defined(EA_9A938E04_44AF_4d5f_B8CC_BE3DF0E08682__INCLUDED_)
#define EA_9A938E04_44AF_4d5f_B8CC_BE3DF0E08682__INCLUDED_

#include "ecp/common/ecp_visual_servo.h"

class ecp_vis_ib_eih_irp6ot : public ecp_visual_servo
{

public:
	ecp_vis_ib_eih_irp6ot(ecp_task& _ecp_task, int step=0);
	virtual ~ecp_vis_ib_eih_irp6ot();

	virtual void next_step_without_constraints();
	virtual void entertain_constraints();
	virtual bool first_step(void);

};
#endif // !defined(EA_9A938E04_44AF_4d5f_B8CC_BE3DF0E08682__INCLUDED_)

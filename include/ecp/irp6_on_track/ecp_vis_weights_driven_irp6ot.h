///////////////////////////////////////////////////////////
//  ecp_vis_weights_driven_irp6ot.h
//  Implementation of the Class ecp_vis_weights_driven_irp6ot
//  Created on:      04-sie-2008 14:25:49
//  Original author: tkornuta
///////////////////////////////////////////////////////////

#if !defined(EA_0ECAB625_4FB6_4695_B581_8BCA0028CF30__INCLUDED_)
#define EA_0ECAB625_4FB6_4695_B581_8BCA0028CF30__INCLUDED_

#include "ecp/common/ecp_visual_servo_manager.h"

class ecp_vis_weights_driven_irp6ot : public ecp_visual_servo_manager
{

public:
	ecp_vis_weights_driven_irp6ot(ecp_task& _ecp_task, int step=0);
	virtual ~ecp_vis_weights_driven_irp6ot();

	virtual void initalize_switching_parameters();
	virtual bool first_step(void);
	virtual bool next_step(void);

};
#endif // !defined(EA_0ECAB625_4FB6_4695_B581_8BCA0028CF30__INCLUDED_)

///////////////////////////////////////////////////////////
//  ecp_visual_servo_manager.h
//  Implementation of the Class ecp_visual_servo_manager
//  Created on:      04-sie-2008 14:24:29
//  Original author: tkornuta
///////////////////////////////////////////////////////////

#if !defined(EA_372F46B1_372D_4660_A605_52297559E64B__INCLUDED_)
#define EA_372F46B1_372D_4660_A605_52297559E64B__INCLUDED_

#include "ecp_visual_servo.h"

class ecp_visual_servo_manager : public ecp_visual_servo
{

public:
	ecp_visual_servo_manager();
	virtual ~ecp_visual_servo_manager();
	ecp_visual_servo *m_ecp_visual_servo;

	virtual void initalize_switching_parameters() =0;

private:
	ecp_visual_servo* ecp_visual_servo_list;

};
#endif // !defined(EA_372F46B1_372D_4660_A605_52297559E64B__INCLUDED_)

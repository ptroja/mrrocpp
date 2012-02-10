///////////////////////////////////////////////////////////
//  ecp_visual_servo_manager.h
//  Implementation of the Class ecp_visual_servo_manager
//  Created on:      04-sie-2008 14:24:29
//  Original author: tkornuta
///////////////////////////////////////////////////////////

/*!
 * \file ecp_visual_servo_manager.h
 * \brief Abstract class as a pattern for implementing swiching/agregating basic visual servos.
 * - class declaration
 * \author tkornuta/mstaniak
 * \date 04.08.2008
 */

#if !defined(EA_372F46B1_372D_4660_A605_52297559E64B__INCLUDED_)
#define EA_372F46B1_372D_4660_A605_52297559E64B__INCLUDED_

#include "ecp_g_visual_servo.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * \class ecp_visual_servo_manager
 * \brief Abstract class as a pattern for implementing swiching/agregating basic visual servos.
 * \author tkornuta/mstaniak
 */

class ecp_visual_servo_manager : public ecp_visual_servo
{

public:
	ecp_visual_servo *m_ecp_visual_servo;
	/*!
	 * Constructor.
	 */
	ecp_visual_servo_manager(common::task::task& _ecp_task, int step = 0);
	/*!
	 * Destructor.
	 */
	virtual ~ecp_visual_servo_manager();

	virtual void initalize_switching_parameters() =0;

private:
	ecp_visual_servo* ecp_visual_servo_list;

};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif // !defined(EA_372F46B1_372D_4660_A605_52297559E64B__INCLUDED_)

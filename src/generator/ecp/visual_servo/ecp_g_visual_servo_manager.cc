///////////////////////////////////////////////////////////
//  ecp_visual_servo_manager.cpp
//  Implementation of the Class ecp_visual_servo_manager
//  Created on:      04-sie-2008 14:24:29
//  Original author: tkornuta
///////////////////////////////////////////////////////////

/*!
 * \file ecp_visual_servo_manager.cc
 * \brief Abstract class as a pattern for implementing swiching/agregating basic visual servos.
 * - method definition
 * \author tkornuta/mstaniak
 * \date 04.08.2008
 */

#include "ecp_g_visual_servo_manager.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

ecp_visual_servo_manager::ecp_visual_servo_manager(common::task::task& _ecp_task, int step) :
		ecp_visual_servo(_ecp_task)
{

}

ecp_visual_servo_manager::~ecp_visual_servo_manager()
{

}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


/*
 * defines.h
 *
 *  Created on: Aug 24, 2010
 *      Author: mboryn
 */

#ifndef VS_DEFINES_H_
#define VS_DEFINES_H_

#define ROBOT_OT
//#define ROBOT_P

#ifdef ROBOT_P
#include "robot/irp6p_m/const_irp6p_m.h"
#define ROBOT_NAME_MB lib::irp6p_m::ROBOT_NAME
#endif

#ifdef ROBOT_OT
#include "robot/irp6ot_m/const_irp6ot_m.h"
#define ROBOT_NAME_MB lib::irp6ot_m::ROBOT_NAME
#endif

#endif /* VS_DEFINES_H_ */

/*
 * bcl_types.h
 *
 *  Created on: Jul 13, 2010
 *      Author: kszkudla
 */

#ifndef BCL_TYPES_H_
#define BCL_TYPES_H_

#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"

#include "../servovision/visual_servo_types.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

typedef struct {
	double x;
	double y;

} regions;

typedef struct {
	bool a;
	int x;
}__attribute__((__packed__)) init;

typedef ecp_mp::sensor::fradia_sensor<lib::empty_t, regions> bcl_fradia_sensor;
//typedef ecp_mp::sensor::fradia_sensor<visual_servo_types::image_based_configuration, visual_servo_types::image_based_reading> bcl_fradia_sensor;

}

}

}

}

#endif /* BCL_TYPES_H_ */

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
	bool code_found;
	int num_found;
	double x_k0;
	double y_k0;
	double w_k0;
	double h_k0;
	double x_k1;
	double y_k1;
	double w_k1;
	double h_k1;
	double x_k2;
	double y_k2;
	double w_k2;
	double h_k2;
	double x_k3;
	double y_k3;
	double w_k3;
	double h_k3;
	double x_k4;
	double y_k4;
	double w_k4;
	double h_k4;


} regions;


typedef ecp_mp::sensor::fradia_sensor<lib::empty_t, regions> bcl_fradia_sensor;
//typedef ecp_mp::sensor::fradia_sensor<visual_servo_types::image_based_configuration, visual_servo_types::image_based_reading> bcl_fradia_sensor;

}

}

}

}

#endif /* BCL_TYPES_H_ */

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


#define IRP6_OT
//#define IRP6_P
//#define JOINT
#define EULER

#define MP_2_ECP_STRING_SIZE 300
#define VEC_POS 27

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

/**
 * Structure used in communication between MRROC++ and FraDIA
 */
typedef struct {
	bool code_found;
	int num_found;
	double x_k0;//Code center X
	double y_k0;//Code center Y
	double r_k0;//Code radius
//		double w_k0;//Code width
//		double h_k0;//Code height
	double x_k1;
	double y_k1;
	double r_k1;
//		double w_k1;
//		double h_k1;
	double x_k2;
	double y_k2;
	double r_k2;
//		double w_k2;
//		double h_k2;
	double x_k3;
	double y_k3;
	double r_k3;
//		double w_k3;
//		double h_k3;
} fradia_regions;


/**
 * Structure used in communication between ECP and MP
 */
typedef struct {
	double x;
	double y;
	double r;
//	double w;
//	double h;

} mrrocpp_regions;

/**
 * Definition of FraDIA sensor type with specified structures
 */
typedef ecp_mp::sensor::fradia_sensor<lib::empty_t, fradia_regions> bcl_fradia_sensor;
//typedef ecp_mp::sensor::fradia_sensor<visual_servo_types::image_based_configuration, visual_servo_types::image_based_reading> bcl_fradia_sensor;

}

}

}

}

#endif /* BCL_TYPES_H_ */

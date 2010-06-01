/*
 * generator/ecp_g_vis_ib_eih_planar_irp6ot.h
 *
 *  Created on: Dec 9, 2008
 *      Author: pwilkows
 */

#ifndef ECP_G_ROTATE_GRIPPER_H_
#define  ECP_G_ROTATE_GRIPPER_H_

#include <iostream>
#include <string.h>
#include <math.h>

#include "ecp/common/generator/ecp_generator.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {

/*!
 * \enum HD_READING
 * \brief Types commands get from PW_HaarDetect task.
 */
typedef enum { HD_SOLUTION_NOTFOUND, HD_SOLUTION_FOUND } HD_READING;

//Obraz fradii dla rotate_gripper
typedef struct _hd_angle {
	HD_READING reading_state;
	float angle;
} hd_angle_t;
/*!
* \brief Types commands sent to PW_HaarDetect task.
*/
typedef enum _HD_MODE
{
WITHOUT_ROTATION, PERFORM_ROTATION
} hd_mode_t;
typedef ecp_mp::sensor::fradia_sensor<lib::empty_t, hd_angle_t, hd_mode_t> fradia_sensor_haar_detect;

class ecp_g_rotate_gripper: public common::generator::generator {
	//Wirtualny sensor
	fradia_sensor_haar_detect *vsp_fradia;
	double speed;
	lib::trajectory_description td;
	bool lastStep;
//	FILE* research;

public:
	ecp_g_rotate_gripper(common::task::task& _ecp_task, double speed);
	virtual ~ecp_g_rotate_gripper();
	virtual bool first_step(void);
	virtual bool next_step();
};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* generator/ecp_g_rotate_gripper_H_ */



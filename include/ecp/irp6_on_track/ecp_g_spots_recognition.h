/*
 * ecp_g_spots_recognition.h
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#ifndef ECP_G_SPOTS_RECOGNITION_H_
#define ECP_G_SPOTS_RECOGNITION_H_


#include "ecp/common/ecp_g_smooth.h"

//fradia
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

class ecp_spots_generator : public ecp_smooth_generator
{
	ECP_VSP_MSG comm_struct;
	ecp_mp_cvfradia_sensor * sensor;
	double tool_to_ground[16], plate_to_tool[16];
	SENSOR_IMAGE calib_data;

	double vec_1[3], vec_2[3], vec_3[3], vec_4[3];

  public:
	ecp_spots_generator(ecp_task& _ecp_task);
	bool first_step();
	bool next_step();

	void get_pic();
	void get_frame();
	void save_position();
};

#endif /* ECP_G_SPOTS_RECOGNITION_H_ */

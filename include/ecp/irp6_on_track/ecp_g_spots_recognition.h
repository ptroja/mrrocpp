/*
 * ecp_g_spots_recognition.h
 *
 *  Created on: Aug 26, 2008
 *      Author: ghard
 */

#ifndef ECP_G_SPOTS_RECOGNITION_H_
#define ECP_G_SPOTS_RECOGNITION_H_

//fradia
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include "ecp/irp6_on_track/ecp_camera_to_tool.h"

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

class spots : public common::generator::base
{
	lib::ECP_VSP_MSG comm_struct;
	ecp_mp::sensor::cvfradia * sensor;
	double tce[12], teg[12], tcg[12];
	long no_of_tcg_in_one;
	CameraToTool * c;
	lib::SENSOR_IMAGE calib_data;

	//double vec_1[4], vec_2[4], vec_3[4], vec_4[4];
	short iter; //0 - zero, 1 - one, 2 - many ;)

  public:
	spots(common::task::base& _ecp_task);
	bool first_step();
	bool next_step();

	void get_pic();
	void get_frame();
	void compute_TCE();
	void compute_TCG();
	void save_position();
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SPOTS_RECOGNITION_H_ */

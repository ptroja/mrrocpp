/*
 * ecp_g_pb_eih.h
 *
 *  Created on: Mar 18, 2010
 *      Author: mboryn
 */

#ifndef ECP_G_PB_EIH_H_
#define ECP_G_PB_EIH_H_

#include "ecp_g_visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

typedef struct pb_object_position_t
{
	/** Shoud be equal to sizeof(pb_object_position_t) */
	size_t size;

	bool tracking;

	/** Translation of the object with respect to the camera. [0]: X, [1]: Y, [2]: Z */
	double translation[3];

	/** Rotation around camera's axes: X,Y,Z. */
	double rotation[3];
} pb_object_position;

class ecp_g_pb_eih: public mrrocpp::ecp::common::generator::visual_servo <4, 4>
{
public:
			ecp_g_pb_eih(mrrocpp::ecp::common::task::task & _ecp_task, ecp_mp::sensor::fradia_sensor <
					pb_object_position> *vsp_fradia, visual_servo_regulator <4, 4> * regulator);
	virtual ~ecp_g_pb_eih();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual bool first_step();
	virtual bool next_step();
protected:
	/**
	 * Check if frame is within constraints.
	 */
	bool isArmFrameOk(const lib::Homog_matrix& arm_frame);
private:
	static const char configSectionName[];
	ecp_mp::sensor::fradia_sensor <visual_object_tracker> *vsp_fradia;

	/** Current effector frame*/
	lib::Homog_matrix currentFrame;

	/** Camera frame with respect to effector frame */
	lib::Homog_matrix e_T_cFrame;

	double max_v, max_a;

	Eigen::Matrix <double, 3, 1> prev_u;
	double delta_t;

};

} // namespace generator

}

}

}

#endif /* ECP_G_PB_EIH_H_ */

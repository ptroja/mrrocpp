/*
 * $Id$
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#ifndef ECP_G_IB_EIH_H_
#define ECP_G_IB_EIH_H_

#include "ecp_g_visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

typedef struct object_tracker_t
{
	bool reached;
	bool tracking;
	int x;
	int y;
	int z;
} object_tracker;

class ecp_g_ib_eih: public visual_servo
{
public:
	ecp_g_ib_eih(mrrocpp::ecp::common::task::task & _ecp_task, ecp_mp::sensor::fradia_sensor<object_tracker> *vsp_fradia, visual_servo_regulator * regulator);
	virtual ~ecp_g_ib_eih();
	virtual bool first_step();
	virtual bool next_step();

	static const char configSectionName[];
protected:
	/**
	 * Check if frame is within constraints.
	 */
	bool isArmFrameOk(const lib::Homog_matrix& arm_frame);
private:
	ecp_mp::sensor::fradia_sensor<object_tracker> *vsp_fradia;

	/** Current effector frame*/
	lib::Homog_matrix currentFrame;

	/** Camera frame with respect to effector frame */
	lib::Homog_matrix e_T_cFrame;
	double currentGripperCoordinate;
	bool currentFrameSaved;

	double max_v, max_a;

	boost::numeric::ublas::vector<double> prev_u;
	double delta_t;
};

/** @} */// ecp_g_ib_eih

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_IB_EIH_H_ */

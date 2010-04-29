/*
 * $Id$
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#ifndef ECP_G_IB_EIH_H_
#define ECP_G_IB_EIH_H_

#include <Eigen/Core>

#include "kinematics/irp6_on_track/kinematic_model_irp6ot_with_wrist.h"

#include "ecp_g_visual_servo.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

class ecp_g_ib_eih: public visual_servo<4, 4>
{
public:
	ecp_g_ib_eih(mrrocpp::ecp::common::task::task & _ecp_task, ecp_mp::sensor::fradia_sensor<visual_object_tracker, char> *vsp_fradia, visual_servo_regulator<4, 4> * regulator);
	virtual ~ecp_g_ib_eih();
	virtual bool first_step();
	virtual bool next_step();

	static const char configSectionName[];

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	/**
	 * Check if frame is within constraints.
	 */
	bool isArmFrameOk(const lib::Homog_matrix& arm_frame);
private:
	ecp_mp::sensor::fradia_sensor<visual_object_tracker, char> *vsp_fradia;

	/** Current effector frame*/
	lib::Homog_matrix currentFrame;

	/** Camera frame with respect to effector frame */
	lib::Homog_matrix e_T_cFrame;
	double currentGripperCoordinate;
	bool currentFrameSaved;

	double max_v, max_a;
	double stop_v, stop_a;
	double stop_e_translation, stop_e_rotation;

	Eigen::Matrix<double, 3, 1> prev_u;
	double delta_t;

	mrrocpp::kinematics::common::kinematic_model *kinematic;
	lib::JointArray local_desired_joints, local_current_joints;
};

/** @} */// ecp_g_ib_eih

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_IB_EIH_H_ */

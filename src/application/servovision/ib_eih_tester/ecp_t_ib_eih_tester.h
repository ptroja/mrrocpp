/*
 * $Id: ecp_t_ib_eih_tester.h 3480 2010-01-08 18:48:29Z mboryn $
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#ifndef ECP_T_IB_EIH_TESTER_H_
#define ECP_T_IB_EIH_TESTER_H_

#include "ecp/common/task/ecp_task.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "../ecp_g_ib_eih.h"
#include "../visual_servo_regulator_p.h"
#include "ecp/common/generator/ecp_g_force.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

/** @addtogroup servovision Visual servomechanism implementation
 *  @ingroup application
 *  IB EIH Servovision tester
 *  @{
 */

/**
 *
 */
class ecp_t_ib_eih_tester: public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_ib_eih_tester(mrrocpp::lib::configurator& _configurator);
	virtual ~ecp_t_ib_eih_tester();
	void main_task_algorithm(void);
protected:
	ecp_mp::sensor::fradia_sensor <ecp::common::generator::visual_object_tracker, char> *vsp_fradia;
	mrrocpp::ecp::common::generator::smooth * smooth_gen;
	mrrocpp::ecp::common::generator::ecp_g_ib_eih * g_ib_eih;
	mrrocpp::ecp::common::generator::visual_servo_regulator <4, 4> * regulator;

	/** Moves (using smooth generator) to initial position. */
	void moveToInitialPosition();

	static const double initialPositionJoints[MAX_SERVOS_NR];

	common::generator::tff_gripper_approach* gagen;
	common::generator::bias_edp_force* befgen; //calibration of force

};

/** @} */// ecp_t_ib_eih_tester

} // namespace task

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_T_IB_EIH_TESTER_H_ */

/*
 * ecp_t_objectfollower_pb_sac.h
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_OBJECTFOLLOWER_PB_SAC_H_
#define ECP_T_OBJECTFOLLOWER_PB_SAC_H_

#include "base/ecp/ecp_task.h"
#include <boost/shared_ptr.hpp>
#include "base/lib/logger.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "../single_visual_servo_manager.h"
#include "../pb_sac_visual_servo.h"
#include "../cubic_constraint.h"
#include "../object_reached_termination_condition.h"
#include "../visual_servo_regulator_p.h"

using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using boost::shared_ptr;
using namespace mrrocpp::ecp::servovision;

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class ecp_t_objectfollower_pb_sac : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_objectfollower_pb_sac(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_objectfollower_pb_sac();
	void main_task_algorithm();
protected:
	shared_ptr <visual_servo_regulator> reg;
	shared_ptr <single_visual_servo_manager> sm;
	shared_ptr <visual_servo> vs;
};

/** @} */

} // namespace task

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_T_OBJECTFOLLOWER_PB_SAC_H_ */

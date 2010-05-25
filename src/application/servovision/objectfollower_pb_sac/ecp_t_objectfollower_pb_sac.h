/*
 * ecp_t_objectfollower_pb_sac.h
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_OBJECTFOLLOWER_PB_SAC_H_
#define ECP_T_OBJECTFOLLOWER_PB_SAC_H_

#include "ecp/common/task/ecp_task.h"
#include <boost/shared_ptr.hpp>
#include "../logger.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
//#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "../simple_visual_servo_manager.h"
#include "../pb_sac_visual_servo.h"
#include "../cubic_constraint.h"
#include "../object_reached_termination_condition.h"
#include "../visual_servo_regulator_p.h"

using mrrocpp::ecp::common::generator::simple_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo;
using mrrocpp::ecp::common::generator::visual_servo_regulator;
using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

class ecp_t_objectfollower_pb_sac : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_objectfollower_pb_sac(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_objectfollower_pb_sac();
	void main_task_algorithm();
protected:
	shared_ptr <visual_servo_regulator> reg;
	shared_ptr <simple_visual_servo_manager> sm;
	shared_ptr <visual_servo> vs;
};

} // namespace task

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_T_OBJECTFOLLOWER_PB_SAC_H_ */

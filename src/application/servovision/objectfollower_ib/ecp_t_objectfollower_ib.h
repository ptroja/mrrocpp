/*
 * ecp_t_objectfollower_ib.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_OBJECTFOLLOWER_IB_H_
#define ECP_T_OBJECTFOLLOWER_IB_H_

#include "ecp/common/task/ecp_task.h"
#include <boost/shared_ptr.hpp>
#include "lib/logger.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
//#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "../simple_visual_servo_manager.h"
#include "../ib_eih_visual_servo.h"
#include "../visual_servo_regulator_p.h"
#include "../cubic_constraint.h"
#include "../object_reached_termination_condition.h"

using mrrocpp::ecp::common::generator::smooth;
using mrrocpp::ecp::common::generator::simple_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo;
using mrrocpp::ecp::common::generator::visual_servo_regulator;
using mrrocpp::ecp::common::generator::object_reached_termination_condition;
using mrrocpp::ecp::common::generator::termination_condition;
using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

/** @addtogroup servovision
 *  @{
 */

class ecp_t_objectfollower_ib: public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_objectfollower_ib(mrrocpp::lib::configurator& configurator);
	virtual ~ecp_t_objectfollower_ib();
	void main_task_algorithm(void);

protected:
	shared_ptr<smooth> smooth_gen;
	shared_ptr<simple_visual_servo_manager> sm;
	shared_ptr<visual_servo> vs;
	shared_ptr<visual_servo_regulator> reg;
	shared_ptr<termination_condition> term_cond;

	/** Moves (using smooth generator) to initial position. */
	void moveToInitialPosition();

	static const double initial_position_joints[MAX_SERVOS_NR];
};

/** @} */

}//namespace task

}

}

}

#endif /* ECP_T_OBJECTFOLLOWER_IB_H_ */

/*
 * ecp_t_objectfollower_pb.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_OBJECTFOLLOWER_PB_H_
#define ECP_T_OBJECTFOLLOWER_PB_H_

#include "base/ecp/ecp_task.h"
#include <boost/shared_ptr.hpp>
#include "base/lib/logger.h"
#include "../simple_visual_servo_manager.h"
#include "../pb_eih_visual_servo.h"
#include "../cubic_constraint.h"
#include "../object_reached_termination_condition.h"
#include "../visual_servo_regulator_p.h"

using mrrocpp::ecp::common::generator::simple_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using namespace mrrocpp::ecp::servovision;
using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

/** @addtogroup servovision
 *  @{
 */

class ecp_t_objectfollower_pb: public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_objectfollower_pb(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_objectfollower_pb();
	void main_task_algorithm(void);

protected:
	shared_ptr<visual_servo_regulator> reg;
	shared_ptr<simple_visual_servo_manager> sm;
	shared_ptr<visual_servo> vs;
	shared_ptr<termination_condition> term_cond;
};

/** @} */

}//namespace task

}

}

}

#endif /* ECP_T_OBJECTFOLLOWER_PB_H_ */

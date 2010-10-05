/*
 * ecp_t_vs_tutorial.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_OBJECTFOLLOWER_IB_H_
#define ECP_T_OBJECTFOLLOWER_IB_H_

#include "base/ecp/ecp_task.h"
#include <boost/shared_ptr.hpp>
#include "base/lib/logger.h"
#include "../single_visual_servo_manager.h"
#include "../ib_eih_visual_servo.h"
#include "../visual_servo_regulator_p.h"
#include "../cubic_constraint.h"
#include "../object_reached_termination_condition.h"
#include "../timeout_termination_condition.h"

using mrrocpp::ecp::common::generator::single_visual_servo_manager;
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

/**
 *
 */
class ecp_t_vs_tutorial: public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_vs_tutorial(mrrocpp::lib::configurator& configurator);
	virtual ~ecp_t_vs_tutorial();
	void main_task_algorithm(void);

protected:

	void move_visual_servo();

	//shared_ptr<smooth> smooth_gen;
	shared_ptr<single_visual_servo_manager> sm;
	shared_ptr<visual_servo> vs;
	shared_ptr<visual_servo_regulator> reg;
	shared_ptr<termination_condition> object_reached_term_cond;
	shared_ptr<termination_condition> timeout_term_cond;

	std::string vs_config_section_name;
};

/** @} */

}//namespace task

}

}

}

#endif /* ECP_T_OBJECTFOLLOWER_IB_H_ */

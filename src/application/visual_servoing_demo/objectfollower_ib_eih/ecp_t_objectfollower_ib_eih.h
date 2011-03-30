/*
 * ecp_t_objectfollower_ib_eih.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_OBJECTFOLLOWER_IB_EIH_H_
#define ECP_T_OBJECTFOLLOWER_IB_EIH_H_

#include <boost/shared_ptr.hpp>
#include "base/ecp/ecp_task.h"
#include "base/lib/logger.h"
#include "application/visual_servoing/visual_servoing.h"

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
class ecp_t_objectfollower_ib_eih: public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_objectfollower_ib_eih(mrrocpp::lib::configurator& configurator);

	void main_task_algorithm(void);

protected:
	//shared_ptr<smooth> smooth_gen;
	shared_ptr<single_visual_servo_manager> sm;
	shared_ptr<visual_servo> vs;
	shared_ptr<visual_servo_regulator> reg;
	shared_ptr<termination_condition> term_cond;
};

/** @} */

}//namespace task

}

}

}

#endif /* ECP_T_OBJECTFOLLOWER_IB_EIH_H_ */

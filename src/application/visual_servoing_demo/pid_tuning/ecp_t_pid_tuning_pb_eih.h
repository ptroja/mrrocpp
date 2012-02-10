/*
 * ecp_t_pid_tuning_pb_eih.h
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_PID_TUNING_PB_EIH_H_
#define ECP_T_PID_TUNING_PB_EIH_H_

#include <boost/shared_ptr.hpp>
#include "base/ecp/ecp_task.h"
#include "base/lib/logger.h"
#include "application/visual_servoing/visual_servoing.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"

using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using namespace mrrocpp::ecp::servovision;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

/** @addtogroup servovision
 *  @{
 */

class ecp_t_pid_tuning_pb_eih : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_pid_tuning_pb_eih(mrrocpp::lib::configurator& config);

	void main_task_algorithm(void);

protected:
	boost::shared_ptr<regulator_p> reg;
	boost::shared_ptr<single_visual_servo_manager> sm;
	boost::shared_ptr<visual_servo> vs;
	boost::shared_ptr<object_reached_termination_condition> obj_reached_term_cond;
	boost::shared_ptr<timeout_termination_condition> timeout_term_cond;

private:
	// gains for P regulator
	double k_p_min, k_p_max, k_p_step;
	// distance from target position
	double step_distance;

	/** Axis of measurement */
	int current_axis;

	int regulator_axis;

//	lib::Homog_matrix object_reached_position, starting_position;

	std::vector<double> object_reached_position, starting_position;

	boost::shared_ptr<generator::newsmooth> newsmooth_gen;
};

/** @} */

}//namespace task

}

}

}

#endif /* ECP_T_OBJECTFOLLOWER_PB_H_ */

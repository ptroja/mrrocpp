/*
 * ecp_t_conveyor_uniform_acceleration.h
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_CONVEYOR_UNIFORM_ACCELERATION_H_
#define ECP_T_CONVEYOR_UNIFORM_ACCELERATION_H_

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_task.h"
#include "ecp_g_conveyor_uniform_acceleration.h"

using boost::shared_ptr;
using mrrocpp::ecp::common::generator::ecp_g_conveyor_uniform_acceleration;

namespace mrrocpp {

namespace ecp {

namespace conveyor {

namespace task {

class ecp_t_conveyor_uniform_acceleration : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_conveyor_uniform_acceleration(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_conveyor_uniform_acceleration();
	void main_task_algorithm(void);
private:
	boost::shared_ptr<ecp_g_conveyor_uniform_acceleration> uniform_acceleration_gen;
};

} //namespace

}

}

}

#endif /* ECP_T_CONVEYOR_UNIFORM_ACCELERATION_H_ */

/*
 * ecp_t_trapezoid_task.h
 *
 *  Created on: 13-01-2011
 *      Author: mboryn
 */

#ifndef ECP_T_TRAPEZOID_TASK_H_
#define ECP_T_TRAPEZOID_TASK_H_

#include <boost/shared_ptr.hpp>
#include "base/ecp/ecp_task.h"
#include "ecp_g_trapezoid_generator.h"
#include "generator/ecp/ecp_g_newsmooth.h"

namespace mrrocpp {

namespace ecp {

namespace trapezoid {

class trapezoid_task : public mrrocpp::ecp::common::task::task
{
public:
	trapezoid_task(lib::configurator &config);
	virtual ~trapezoid_task();
	void main_task_algorithm();

private:
	mrrocpp::ecp::common::generator::newsmooth* cvgenjoint;
//	boost::shared_ptr<trapezoid_generator> trapezoid_gen;
};

}

}

}

#endif /* ECP_T_TRAPEZOID_TASK_H_ */

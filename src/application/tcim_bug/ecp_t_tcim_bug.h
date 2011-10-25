/*
 * ecp_t_tcim_bug.h
 *
 *  Created on: 07-06-2011
 *      Author: mboryn
 */

#ifndef ECP_T_TCIM_BUG_H_
#define ECP_T_TCIM_BUG_H_

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_task.h"
#include "ecp_g_tcim_bug.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

using mrrocpp::ecp::common::generator::ecp_g_tcim_bug;

class ecp_t_tcim_bug : public task
{
public:
	ecp_t_tcim_bug(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_tcim_bug();
	virtual void main_task_algorithm();
private:
	boost::shared_ptr<ecp_g_tcim_bug> gen;
};

}

}

}

}

#endif /* ECP_T_TCIM_BUG_H_ */

/*
 * ecp_t_mboryn.h
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#ifndef ECP_T_MBORYN_H_
#define ECP_T_MBORYN_H_

#include "ecp/common/ecp_task.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp_g_mboryn.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace task {

class ecp_t_mboryn: public mrrocpp::ecp::common::task::task {
public:
	ecp_t_mboryn(mrrocpp::lib::configurator& _configurator);
	virtual ~ecp_t_mboryn();
	void main_task_algorithm(void);
protected:
	generator::ecp_g_mboryn* ecp_g_mboryn_;

};

}

}

}

}

#endif /* ECP_T_MBORYN_H_ */

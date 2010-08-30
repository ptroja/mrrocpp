/*
 * ecp_t_conveyor_test.h
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#ifndef ECP_T_CONVEYOR_TEST_H_
#define ECP_T_CONVEYOR_TEST_H_

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_task.h"
#include "ecp_g_conveyor_sinus.h"

using boost::shared_ptr;
using mrrocpp::ecp::common::generator::ecp_g_conveyor_sinus;

namespace mrrocpp {

namespace ecp {

namespace conveyor {

namespace task {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class ecp_t_conveyor_test : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_conveyor_test(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_conveyor_test();
	void main_task_algorithm(void);
private:
	boost::shared_ptr<ecp_g_conveyor_sinus> sinus_gen;
};

/** @} */

} //namespace

}

}

}

#endif /* ECP_T_CONVEYOR_TEST_H_ */

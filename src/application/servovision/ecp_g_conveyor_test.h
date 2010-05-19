/*
 * ecp_g_conveyor_test.h
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#ifndef ECP_G_CONVEYOR_TEST_H_
#define ECP_G_CONVEYOR_TEST_H_

#include "ecp_generator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

class ecp_g_conveyor_test : public mrrocpp::ecp::common::generator::generator
{
public:
	ecp_g_conveyor_test(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name);
	virtual ~ecp_g_conveyor_test();

	bool first_step();
	bool next_step();

private:
	double t;
	double dt;
	int motion_steps;
};

}

}

}

}

#endif /* ECP_G_CONVEYOR_TEST_H_ */

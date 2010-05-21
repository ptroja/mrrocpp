/*
 * ecp_g_conveyor_sinus.h
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#ifndef ECP_G_CONVEYOR_SINUS_H_
#define ECP_G_CONVEYOR_SINUS_H_

#include <string>

#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

class ecp_g_conveyor_sinus : public mrrocpp::ecp::common::generator::generator
{
public:
	ecp_g_conveyor_sinus(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name);
	virtual ~ecp_g_conveyor_sinus();

	bool first_step();
	bool next_step();
private:

	int motion_steps;
	double dt;
	double A;
	double f;
	double t;

	bool initial_position_saved;
	double initial_position;
};

}//namespace

}

}

}

#endif /* ECP_G_CONVEYOR_SINUS_H_ */

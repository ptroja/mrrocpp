/*
 * ecp_g_tcim_bug.h
 *
 *  Created on: 07-06-2011
 *      Author: mboryn
 */

#ifndef ECP_G_TCIM_BUG_H_
#define ECP_G_TCIM_BUG_H_

#include <ctime>
#include <sstream>
#include <iostream>

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

class ecp_g_tcim_bug : public generator
{
public:
	ecp_g_tcim_bug(mrrocpp::ecp::common::task::task & ecp_task);
	virtual ~ecp_g_tcim_bug();

	virtual bool first_step();
	virtual bool next_step();

private:
	int motion_steps;

	struct timespec current_timestamp;
	struct timespec prev_timestamp;
	std::stringstream ss;
	int c;

	lib::Homog_matrix current_position;
	bool current_position_saved;
};

}

}

}

}

#endif /* ECP_G_TCIM_BUG_H_ */

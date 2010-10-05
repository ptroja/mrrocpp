/*
 * ecp_g_conveyor_uniform_acceleration.h
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#ifndef ECP_G_CONVEYOR_UNIFORM_ACCELERATION_H_
#define ECP_G_CONVEYOR_UNIFORM_ACCELERATION_H_

#include <string>

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class ecp_g_conveyor_uniform_acceleration : public mrrocpp::ecp::common::generator::generator
{
public:
	ecp_g_conveyor_uniform_acceleration(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name);
	virtual ~ecp_g_conveyor_uniform_acceleration();

	bool first_step();
	bool next_step();
private:

	int motion_steps;
	double dt;
	double acceleration;
	double current_speed;
	double max_speed;
	double t;

	bool initial_position_saved;
	double current_position;
};

/** @} */

}//namespace

}

}

}

#endif /* ECP_G_CONVEYOR_UNIFORM_ACCELERATION_H_ */

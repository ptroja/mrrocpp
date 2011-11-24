/*
 * ecp_g_trajectoryline.h
 *
 *  Created on: 11-08-2011
 *      Author: mateusz
 */

#ifndef ECP_G_TRAJECTORYLINE_H_
#define ECP_G_TRAJECTORYLINE_H_

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class ecp_g_trajectory_line : public mrrocpp::ecp::common::generator::generator
{
public:
	ecp_g_trajectory_line(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name);
	virtual ~ecp_g_trajectory_line();
	bool first_step();
	bool next_step();
private:

	int motion_steps;
	int motion_steps_value_in_step_no;
	double dt;
	double A;
	double f;
	double t;

	bool initial_position_saved;
	lib::Homog_matrix initial_position;
};

} /* namespace generator */
} /* namespace common */
} /* namespace ecp */
} /* namespace mrrocpp */
#endif /* ECP_G_TRAJECTORYLINE_H_ */

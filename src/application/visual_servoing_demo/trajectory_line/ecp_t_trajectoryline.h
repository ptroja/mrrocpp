/*
 * ecp_t_trajectoryline.h
 *
 *  Created on: 11-08-2011
 *      Author: mateusz
 */

#ifndef ECP_T_TRAJECTORYLINE_H_
#define ECP_T_TRAJECTORYLINE_H_

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_task.h"
#include "ecp_g_trajectoryline.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

using mrrocpp::ecp::common::generator::ecp_g_trajectory_line;

class ecp_t_trajectory_line : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_trajectory_line(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_trajectory_line();
	void main_task_algorithm();

private:
	boost::shared_ptr<ecp_g_trajectory_line> gen;
};

} /* namespace task */
} /* namespace common */
} /* namespace ecp */
} /* namespace mrrocpp */
#endif /* ECP_T_TRAJECTORYLINE_H_ */

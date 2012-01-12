/*
 * ecp_g_sbench.h
 *
 * Author: ptroja
 */

#ifndef ECP_G_SBENCH_H_
#define ECP_G_SBENCH_H_

#include "robot/sbench/ecp_r_sbench.h"
#include "robot/sbench/dp_sbench.h"

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {
namespace generator {

class pin_config : public common::generator::_generator<ecp::sbench::robot>
{
public:
	//! Constructor
	pin_config(task_t & _ecp_task, const lib::sbench::voltage_buffer & _pins_buffer);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();

private:
	//! Pin configuration
	const lib::sbench::voltage_buffer & pin_configuration;
};

} // namespace generator
} // namespace sbench
} // namespace ecp
} // namespace mrrocpp

#endif

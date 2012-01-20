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

class power_supply : public common::generator::_generator<ecp::sbench::robot>
{
public:
	//! Constructor
	power_supply(task_t & _ecp_task, const lib::sbench::power_supply_state & _pins_buffer);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();

private:
	//! Pin configuration
	const lib::sbench::power_supply_state & pin_configuration;
};


class cleaning : public common::generator::_generator<ecp::sbench::robot>
{
public:
	//! Constructor
	cleaning(task_t & _ecp_task, const lib::sbench::cleaning_state & _pins_buffer);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();

private:
	//! Pin configuration
	const lib::sbench::cleaning_state & pin_configuration;
};

} // namespace generator
} // namespace sbench
} // namespace ecp
} // namespace mrrocpp

#endif

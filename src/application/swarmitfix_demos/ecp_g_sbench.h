/*!
 * @file ecp_g_sbench.h
 * @brief Power supply and pressure control transparent generators - both forward the command prepared by MP to EDP.
 *
 * @date Jan 19, 2012
 * @author tkornuta
 */

#ifndef ECP_G_SBENCH_TRANSPARENT_H_
#define ECP_G_SBENCH_TRANSPARENT_H_

#include "robot/sbench/ecp_r_sbench.h"
#include "robot/sbench/dp_sbench.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {
namespace generator {

/*!
 * @brief Power supply generator forwards the command prepared by MP to EDP regarding the control of the power supply in bench pins.
 * @note Ends after the first_step execution.
 *
 * @author tkornuta
 * @ingroup generators
 */
class power_supply : public common::generator::_generator <ecp::sbench::robot>
{
public:
	/*!
	 * Empty.
	 * @param _ecp_task ecp task object reference.
	 */
	power_supply(task_t & _ecp_task);

	/*!
	 * Passes the received MP command to EDP.
	 */
	bool first_step();

	/*!
	 * Empty - returns false.
	 */
	bool next_step();

private:
	/*!
	 * Command received from MP.
	 */
	lib::sbench::power_supply_state mp_ecp_power_supply_command;
};


/*!
 * @brief Cleaning generator forwards the command prepared by MP to EDP regarding the control of the pressure in the FESTO cleaning valves.
 * @note Ends after the first_step execution.
 *
 * @author tkornuta
 * @ingroup generators
 */
class cleaning : public common::generator::_generator <ecp::sbench::robot>
{
public:
	/*!
	 * Empty.
	 * @param _ecp_task ecp task object reference.
	 */
	cleaning(task_t & _ecp_task);

	/*!
	 * Passes the received MP command to EDP.
	 */
	bool first_step();

	/*!
	 * Empty - returns false.
	 */
	bool next_step();

private:
	/*!
	 * Command received from MP.
	 */
	lib::sbench::cleaning_state mp_ecp_cleaning_command;
};


} // namespace generator
} // namespace sbench
} // namespace ecp
} // namespace mrrocpp

#endif

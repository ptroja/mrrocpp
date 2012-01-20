/*!
 * @file ecp_t_sbench.h
 * @brief Transparent task class declaration.
 *
 * @date Jan 19, 2012
 * @author tkornuta
 */

#if !defined(ECP_T_SBENCH_TRANSPARENT_H_)
#define ECP_T_SBENCH_TRANSPARENT_H_

#include <boost/shared_ptr.hpp>

#include "robot/shead/ecp_r_shead.h"
#include "base/ecp/ecp_task.h"
#include "ecp_g_sbench.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {
namespace task {

/*!
 * @brief Transparent task used for passing the commands regarding the power supply and chips cleaning commands from MP to EDP.
 * @note Ends after the first_step execution.
 *
 * @author tkornuta
 * @ingroup generators
 */
class transparent : public common::task::_task <ecp::sbench::robot>
{
protected:
	//! Controls the bench power supply.
	boost::shared_ptr <generator::power_supply> g_power_supply;

	//! Controls the bench cleaning.
	boost::shared_ptr <generator::cleaning> g_cleaning;

public:
	//! Created the utilized generators.
	transparent(lib::configurator &_config);

	//! methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

} // namespace task
} // namespace sbench
} // namespace ecp
} // namespace mrrocpp

#endif

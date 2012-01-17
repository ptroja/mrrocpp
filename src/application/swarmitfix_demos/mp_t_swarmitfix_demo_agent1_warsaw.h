/*!
 * @file mp_t_swarmitfix_demo_agent1_warsaw.h
 *
 * @date Dec 29, 2011
 * @author tkornuta
 */

#ifndef SWARMITFIXDEMOAGENT1WARSAW_H_
#define SWARMITFIXDEMOAGENT1WARSAW_H_

#include "mp_t_swarmitfix_demo_base.h"

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  A swarmitfix demo base class.
 *  @{
 */

/*!
 * @brief Agent1 SwarmItFIX final demo executed in Warsaw.
 *
 * @author tkornuta
 * @date Dec 29, 2011
 */
class swarmitfix_demo_agent1_warsaw : public mrrocpp::mp::task::swarmitfix_demo_base
{
public:
	//! Calls the base class constructor.
	swarmitfix_demo_agent1_warsaw(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	// Executes the plan for supporting of a wooden plate in few, learned positions.
	void main_task_algorithm(void);

	//! Empty.
	virtual ~swarmitfix_demo_agent1_warsaw() { }
};

/** @} */// end of swarmitfix

} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
#endif /* SWARMITFIXDEMOAGENT1WARSAW_H_ */

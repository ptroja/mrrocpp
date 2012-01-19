/*!
 * @file mp_t_demo_agent1_warsaw.h
 *
 * @date Dec 29, 2011
 * @author tkornuta
 */

#ifndef SWARMITFIXDEMOAGENT1WARSAW_H_
#define SWARMITFIXDEMOAGENT1WARSAW_H_

#include "mp_t_demo_base.h"

namespace mrrocpp {
namespace mp {
namespace task {
namespace swarmitfix {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  @{
 */

/*!
 * @brief Agent1 SwarmItFIX final demo executed in Warsaw.
 *
 * @author tkornuta
 * @date Dec 29, 2011
 */
class demo_agent1_warsaw : public mrrocpp::mp::task::swarmitfix::demo_base
{
public:
	//! Calls the base class constructor.
	demo_agent1_warsaw(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	// Executes the plan for supporting of a wooden plate in few, learned positions.
	void main_task_algorithm(void);

	//! Empty.
	virtual ~demo_agent1_warsaw() { }
};

/** @} */// end of swarmitfix

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
#endif /* SWARMITFIXDEMOAGENT1WARSAW_H_ */

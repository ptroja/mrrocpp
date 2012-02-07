/*!
 * @file mp_t_agent1_demo.h
 *
 * @date Jan 20, 2012
 * @author tkornuta
 */

#ifndef SWARMITFIX_AGENT1_DEMO_H_
#define SWARMITFIX_AGENT1_DEMO_H_

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
 * @brief Agent1 SwarmItFIX demo executed in Piaggio.
 *
 * @author tkornuta
 * @date Jan 20, 2012
 */
class agent1_demo : public mrrocpp::mp::task::swarmitfix::demo_base
{
public:
	//! Calls the base class constructor.
	agent1_demo(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	// Executes the plan for supporting of a wooden plate in few, learned positions.
	void main_task_algorithm(void);

	//! Empty.
	virtual ~agent1_demo() { }
};

/** @} */// end of swarmitfix

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
#endif /* SWARMITFIX_AGENT1_DEMO_H_ */

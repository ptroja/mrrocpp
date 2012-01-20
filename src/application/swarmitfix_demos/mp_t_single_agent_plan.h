/*!
 * @file mp_t_agent1_demo.h
 *
 * @date Jan 20, 2012
 * @author tkornuta
 */

#ifndef SWARMITFIX_SINGLE_AGENT_PLAN_H_
#define SWARMITFIX_SINGLE_AGENT_PLAN_H_

#include <boost/shared_ptr.hpp>

#include "base/ecp_mp/ecp_ui_msg.h"
#include "mp_t_demo_base.h"

#include "plan.hxx"

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
class single_agent_demo : public mrrocpp::mp::task::swarmitfix::demo_base
{
public:
	//! Calls the base class constructor.
	single_agent_demo(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	//! Executes the plan for supporting of a wooden plate in few, learned positions.
	void main_task_algorithm(void);

protected:
	//! Plan.
	boost::shared_ptr<Plan> p;

	//! Handle PKM+HEAD plan item.
	void executeCommandItem(const Plan::PkmType::ItemType & pkmCmd);

	//! Handle BENCH+MBASE plan item.
	void executeCommandItem(const Plan::MbaseType::ItemType & smbCmd);

	//! Save modified plan to file
	void save_plan(const Plan & p);

	//! Step-mode execution of pkm item
	lib::UI_TO_ECP_REPLY step_mode(Pkm::ItemType & item);

	//! Step-mode execution of mbase item
	lib::UI_TO_ECP_REPLY step_mode(Mbase::ItemType & item);

	//! Access to plan items at given index
	template <typename T>
	typename T::iterator StateAtInd(int ind, T & items)
	{
		typename T::iterator it = items.begin();

		while ((it != items.end()) && it->ind() != ind)
			++it;

		return it;
	}
};

/** @} */// end of swarmitfix

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */

#endif /* SWARMITFIX_SINGLE_AGENT_PLAN_H_ */

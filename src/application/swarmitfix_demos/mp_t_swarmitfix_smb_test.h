/*!
 * @file mp_t_swarmitfix_smb_test.h
 *
 * @date Jan 17, 2012
 * @author tkornuta
 */

#ifndef MPTSWARMITFIXLEGSTEST_H_
#define MPTSWARMITFIXLEGSTEST_H_

#include "mp_t_swarmitfix_demo_base.h"

namespace mrrocpp {
namespace mp {
namespace task {

class swarmitfix_smb_test : public mrrocpp::mp::task::swarmitfix_demo_base
{
public:
	//! Calls the base class constructor.
	swarmitfix_smb_test(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	// Executes the plan for supporting of a wooden plate in few, learned positions.
	void main_task_algorithm(void);

	//! Empty.
	virtual ~swarmitfix_smb_test() { }
};

} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
#endif /* MPTSWARMITFIXLEGSTEST_H_ */

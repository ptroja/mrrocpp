/*!
 * @file mp_t_smb_test.h
 *
 * @date Jan 17, 2012
 * @author tkornuta
 */

#ifndef MPTSWARMITFIXLEGSTEST_H_
#define MPTSWARMITFIXLEGSTEST_H_

#include "mp_t_demo_base.h"

namespace mrrocpp {
namespace mp {
namespace task {
namespace swarmitfix {

/*!
 *
 */
class smb_test : public mrrocpp::mp::task::swarmitfix::demo_base

{
public:
	//! Calls the base class constructor.
	smb_test(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	// Executes different modes of walking SMB on the bench, depending on the loaded configuration.
	void main_task_algorithm(void);

	//! Empty.
	virtual ~smb_test() { }
};

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
#endif /* MPTSWARMITFIXLEGSTEST_H_ */

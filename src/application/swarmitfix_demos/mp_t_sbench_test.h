/*!
 * @file mp_t_sbench_test.h
 * @brief Class for SBENCH tests.
 *
 * @date Jan 19, 2012
 * @author tkornuta
 */

#ifndef MPTSWARMITFIXLEGSTEST_H_
#define MPTSWARMITFIXLEGSTEST_H_

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
 * @brief Task for testing bench cleaning and power supply.
 *
 * @author tkornuta
 * @date Jan 19, 2012
 */
class sbench_test : public mrrocpp::mp::task::swarmitfix::demo_base

{
public:
	//! Calls the base class constructor.
	sbench_test(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	// Executes different modes of walking SMB on the bench, depending on the loaded configuration.
	void main_task_algorithm(void);

	//! Empty.
	virtual ~sbench_test() { }

};



/** @} */ // end of swarmitfix
} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
#endif /* MPTSWARMITFIXLEGSTEST_H_ */

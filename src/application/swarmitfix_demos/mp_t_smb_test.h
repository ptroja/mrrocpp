/*!
 * @file mp_t_smb_test.h
 * @brief Class for SMB tests.
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

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  @{
 */

/*!
 * @brief Task for testing of the SMB walking on the bench.
 *
 * Performs different SMB monkey tests:
 *  0 - ALL IN/OUT (default)
 *  1 - 1IN -> 2IN -> 3IN
 *  2 - 1IN -> rotation -> 2IN -> rotation -> 3IN -> rotation
 *  3 - Trajectory adequate to the one found by the planners: (1,+60) -> (3,+120) -> (3,-120) -> (1-6)
 *
 * @author tkornuta
 * @date Dec 29, 2011
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

/** @} */ // end of swarmitfix
} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
#endif /* MPTSWARMITFIXLEGSTEST_H_ */

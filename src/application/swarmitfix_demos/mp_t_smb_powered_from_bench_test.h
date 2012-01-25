/*!
 * @file mp_t_smb_powered_from_bench_test.h
 * @brief Class for SMB tests.
 *
 * @date Jan 17, 2012
 * @author tkornuta
 */

#ifndef MPTSWARMITFIXLEGSTEST_H_
#define MP_T_SWARMITFIX_SMB_POWERED_FROM_BENCH_TEST_H_

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
 * @brief Task for testing of the SMB walking on and taking power from the bench.
 *
 * Performs different POWER SMB monkey tests:
 *  0 - power
 *  1 - power + cleaning
 *
 * @author tkornuta
 * @date Jan 19, 2012
 */
class smb_powered_from_bench_test : public mrrocpp::mp::task::swarmitfix::demo_base

{
public:
	//! Calls the base class constructor.
	smb_powered_from_bench_test(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	// Executes different modes of walking SMB on the bench, depending on the loaded configuration.
	void main_task_algorithm(void);

	//! Empty.
	virtual ~smb_powered_from_bench_test() { }
};

/** @} */ // end of swarmitfix
} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
#endif /* MPTSWARMITFIXLEGSTEST_H_ */

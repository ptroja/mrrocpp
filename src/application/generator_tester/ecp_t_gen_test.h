#if !defined(_ECP_T_GEN_TEST_H)
#define _ECP_T_GEN_TEST_H

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "ecp_mp_t_gen_test.h"
#include "ecp_st_const_vel_gen_test.h"

#include "base/ecp/ecp_task.h"
#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class gen_test : public common::task::task
{
protected:

public:
	/**
	 * Constructor.
	 */
	gen_test(lib::configurator &_config);

};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

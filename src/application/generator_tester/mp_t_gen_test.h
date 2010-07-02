#if !defined(__MP_T_GEN_TEST_H)
#define __MP_T_GEN_TEST_H

#include "base/mp/mp.h"
#include "lib/mrmath/mrmath.h"

#include "lib/data_port_headers/tfg.h"
#include "robot/irp6ot_tfg/irp6ot_tfg_const.h"
#include "robot/irp6p_tfg/irp6p_tfg_const.h"
#include "robot/irp6ot_m/irp6ot_m_const.h"
#include "robot/irp6p_m/irp6p_m_const.h"

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <iostream>
#include <string>
#include <sstream>
#include "lib/srlib.h"

#include "ecp_mp_t_gen_test.h"
#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @defgroup gen_test gen_test
 * @ingroup application
 * A gen_test (with active coordinator) QNX test application
 */

class gen_test: public task {
protected:

public:

	/**
	 * Constructor.
	 */
	gen_test(lib::configurator &_config);

	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif

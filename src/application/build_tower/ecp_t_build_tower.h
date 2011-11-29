/*
 * ecp_t_build_tower.h
 *
 *  Created on: 17-11-2011
 *      Author: spiatek
 */

#ifndef ECP_T_BUILD_TOWER_H_
#define ECP_T_BUILD_TOWER_H_

#include "base/lib/logger.h"
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"

#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"

#include "base/ecp/ecp_task.h"
#include "subtask/ecp_mp_st_smooth_file_from_mp.h"
#include "subtask/ecp_st_go.h"

using boost::shared_ptr;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class build_tower : public common::task::task
{
protected:

	common::generator::tff_gripper_approach* gtga;
	common::generator::newsmooth* sg;

	void get_coordinates_from_file(const char*);

public:

	build_tower(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp



#endif /* ECP_T_BUILD_TOWER_H_ */

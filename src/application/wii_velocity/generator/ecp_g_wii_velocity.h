#ifndef ECP_WII_VELOCITY_GENERATOR_H
#define ECP_WII_VELOCITY_GENERATOR_H

#include "generator/ecp/force/ecp_g_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

/** @addtogroup wii_velocity
 *
 *  @{
 */

#define C_0 0.1
#define C_1 0.1
#define C_2 0.1
#define C_3 0.1
#define C_4 0.1
#define C_5 0.1

class wii_velocity : public common::generator::tff_nose_run
{
public:
	wii_velocity(common::task::task& _ecp_task);
	virtual bool first_step();
	virtual bool next_step();
};

/** @} */// end of wii_velocity

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif //ECP_WII_VELOCITY_GENERATOR_H

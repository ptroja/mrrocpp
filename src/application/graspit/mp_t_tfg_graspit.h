#if !defined(__MP_T_TFG_GRASPIT_H)
#define __MP_T_TFG_GRASPIT_H

#include "ecp_mp_tr_graspit.h"

namespace mrrocpp {
namespace mp {
namespace task {

class graspit : public task
{

private:
	ecp_mp::transmitter::TRGraspit* trgraspit;

public:
	graspit(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	// methods for mp template
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif

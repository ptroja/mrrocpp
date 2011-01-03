#if !defined(__MP_T_BIRDHAND_GRASPIT_H)
#define __MP_T_BIRDHAND_GRASPIT_H

#include "base/mp/MP_main_error.h"
#include "ecp_mp_tr_graspit.h"

namespace mrrocpp {
namespace mp {
namespace task {

class graspit : public task
{

private:
	/**
	 * transmitter for communication with GraspIt
	 */
	ecp_mp::transmitter::TRGraspit* trgraspit;

public:
	/**
	 * constructor
	 * @param _config
	 * @return
	 */
	graspit(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	/**
	 * methods for mp template
	 */
	void main_task_algorithm(void);
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif

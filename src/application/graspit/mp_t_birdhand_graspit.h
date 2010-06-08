
#if !defined(__MP_T_BIRDHAND_GRASPIT_H)
#define __MP_T_BIRDHAND_GRASPIT_H

#include "mp/mp.h"
#include "ecp_mp_tr_graspit.h"

namespace mrrocpp {
namespace mp {
namespace task {

class graspit: public task {

	private:
		ecp_mp::transmitter::TRGraspit* trgraspit;

	public:
		graspit(lib::configurator &_config);

		// methods for mp template
		void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif

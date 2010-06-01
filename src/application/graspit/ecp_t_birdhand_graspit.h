#if !defined(_ECP_T_BIRDHAND_GRASPIT_H)
#define _ECP_T_BIRDHAND_GRASPIT_H

#include "ecp/common/task/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {
namespace task {

class bird_hand_test: public common::task::task {
protected:
	//generatory
	common::generator::bird_hand* g_bird_hand;

public:
	// KONSTRUKTORY
	bird_hand_test(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace bird_hand
} // namespace ecp
} // namespace mrrocpp

#endif

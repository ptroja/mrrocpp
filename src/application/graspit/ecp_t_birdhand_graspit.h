#if !defined(_ECP_T_BIRDHAND_GRASPIT_H)
#define _ECP_T_BIRDHAND_GRASPIT_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {
namespace task {

/*!
 * \class bird_hand_test
 * \brief Class for getting commands from MP and moving Bird Hand
 *
 * \author kczajkow
 * \date Jan 05, 2010
 */
class bird_hand_test: public common::task::task {
protected:
	//generatory
	common::generator::bird_hand* g_bird_hand;

public:
	// KONSTRUKTORY
	bird_hand_test(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace bird_hand
} // namespace ecp
} // namespace mrrocpp

#endif

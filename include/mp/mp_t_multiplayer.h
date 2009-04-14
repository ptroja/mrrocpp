#if !defined(__MP_T_MULTIPLAYER_H)
#define _MP_T_MULTIPLAYER_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

class multiplayer : public base
{
	private:
		void move_electron_robot(const lib::playerpos_goal_t &goal);
	public:

		multiplayer(lib::configurator &_config);

		// methods for mp template
		void task_initialization(void);
		void main_task_algorithm(void);

};


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif

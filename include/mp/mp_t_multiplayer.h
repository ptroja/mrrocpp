#if !defined(__MP_T_MULTIPLAYER_H)
#define _MP_T_MULTIPLAYER_H

#include "mp/mp.h"

class mp_task_multiplayer : public mp_task
{
	private:
		bool move_electron_robot(const playerpos_goal_t &goal);
	public:

		mp_task_multiplayer(configurator &_config);

		// methods for mp template
		void task_initialization(void);
		void main_task_algorithm(void);

};

#endif

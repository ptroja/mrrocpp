#if !defined(__MP_T_MULTIPLAYER_H)
#define _MP_T_MULTIPLAYER_H

#include "mp/mp.h"

class mp_task_multiplayer : public mp_task  
{
public:

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

};

#endif

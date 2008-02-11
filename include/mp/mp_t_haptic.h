// -------------------------------------------------------------------------
//                            mp_t_haptic.h
// 
// MP task for two robot haptic device
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_T_HAPTIC_H)
#define __MP_T_HAPTIC_H

#include "mp/mp.h"

class mp_task_haptic : public mp_task  
{
protected:
	bool break_state;
  
public:

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

};

#endif

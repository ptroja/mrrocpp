// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie vision force
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_SB_H)
#define __MP_TASK_SB_H

#include "mp/mp.h"

class mp_task_sb: public mp_task  {
protected:


public:
	
	mp_task_sb(configurator &_config);

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);
  
};

#endif

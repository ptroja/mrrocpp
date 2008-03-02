// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie vision force
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_VIS_NN_H)
#define __MP_TASK_VIS_NN_H

#include "mp/mp.h"

class mp_task_vis_nn: public mp_task  {
protected:

	bool break_state;
public:
	
	mp_task_vis_nn(configurator &_config);

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);
  
};

#endif

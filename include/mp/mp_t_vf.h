// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie vision force
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_VF_H)
#define __MP_TASK_VF_H

#include "mp/mp.h"

class mp_task_vf: public mp_task  {
	
public:

	// methods fo mp template to redefine in concete class
	void task_initialization(void);
	void main_task_algorithm(void);
  
};

#endif

// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - wersja common
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_C_H)
#define __MP_TASK_C_H

#include "mp/mp.h"

class mp_task_c: public mp_task
{
	
public:

	// methods fo mp template to redefine in concete class
	void task_initialization(void);
	void main_task_algorithm(void);
  
};

#endif
